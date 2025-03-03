// To test the node, run the following command:
// ros2 action send_goal /move_to_people roast_interfaces/action/MoveToPeople
// "{'move_to_group': 'true', 'turn_to_group': 'true'}"

#include "roast_people_detection/move_to_people.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp_components/register_node_macro.hpp"

namespace roast_people_detection {
MoveToPeopleActionServer::MoveToPeopleActionServer(
    const rclcpp::NodeOptions &options)
    : Node("move_to_people_action_server", options) {
  using namespace std::placeholders;
  using namespace std::chrono_literals;

  declare_parameter("stop_on_failure", true);
  declare_parameter("loop_rate", 20);

  stop_on_failure_ = get_parameter("stop_on_failure").as_bool();
  loop_rate_ = get_parameter("loop_rate").as_int();
  std::string package_share_directory =
      ament_index_cpp::get_package_share_directory("roast_people_detection");
  behavior_tree_path_ =
      package_share_directory + "/behavior_trees/move_to_group.xml";
  is_goal_active_ = false;

  nav_to_pose_client_ = rclcpp_action::create_client<ClientT>(
      get_node_base_interface(), get_node_graph_interface(),
      get_node_logging_interface(), get_node_waitables_interface(),
      "navigate_to_pose", callback_group_);

  action_server_ = std::make_unique<ActionServer>(
      get_node_base_interface(), get_node_clock_interface(),
      get_node_logging_interface(), get_node_waitables_interface(),
      "move_to_people",
      std::bind(&MoveToPeopleActionServer::move_to_people_group, this));

  subscription_ = this->create_subscription<roast_interfaces::msg::PeopleGroup>(
      "people_info", 1,
      std::bind(&MoveToPeopleActionServer::groupInfoCallback, this,
                std::placeholders::_1));

  // Wait for the navigation action server to start
  while (!nav_to_pose_client_->wait_for_action_server(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Interrupted while waiting for the navigation action "
                   "server. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(),
                "Waiting for the navigation action server to start.");
  }

  // Activate
  action_server_->activate();

  RCLCPP_INFO(this->get_logger(), "Move To People Action Server is active");

  marker_pub_ = this->create_publisher<Marker>("debug/move_to_people", 1);
  marker_.header.frame_id = "map";
  marker_.header.stamp = this->now();
  marker_.ns = "move_to_people";
  marker_.id = 0;
  marker_.type = Marker::CYLINDER;
  marker_.action = Marker::ADD;
  marker_.pose.position.x = 0;
  marker_.pose.position.y = 0;
  marker_.pose.position.z = 1.0;
  marker_.pose.orientation.x = 0.0;
  marker_.pose.orientation.y = 0.0;
  marker_.pose.orientation.z = 0.0;
  marker_.pose.orientation.w = 1.0;
  marker_.scale.x = 0.2;
  marker_.scale.y = 0.2;
  marker_.scale.z = 1.0;
  marker_.color.a = 1.0;
  marker_.color.r = 0.0;
  marker_.color.g = 1.0;
  marker_.color.b = 0.0;
}

MoveToPeopleActionServer::~MoveToPeopleActionServer() {}

void MoveToPeopleActionServer::move_to_people_group() {
  auto goal = action_server_->get_current_goal();
  auto feedback = std::make_shared<ActionT::Feedback>();
  auto result = std::make_shared<ActionT::Result>();

  is_goal_active_ = goal->move_to_group;
  auto topic_wait_time = goal->topic_timeout;

  new_msg_ = false;
  group_detected_ = false;

  // Check if request is valid
  if (!action_server_ || !action_server_->is_server_active()) {
    RCLCPP_WARN(this->get_logger(), "Action Server inactive. Stopping.");
    result->success = false;
    action_server_->succeeded_current(result);
    return;
  }

  if (!is_goal_active_) {
    RCLCPP_INFO(
        this->get_logger(),
        "Exiting since move towards people group has been requested to stop");
    // Cancel navigation related tasks
    this->nav_to_pose_client_->async_cancel_all_goals();
    result->success = true;
    action_server_->succeeded_current(result);
    return;
  }

  rclcpp::WallRate r(loop_rate_);

  if (!new_msg_) {
    RCLCPP_INFO(this->get_logger(),
                "Waiting for people info topic to be published");
    auto start_time = std::chrono::steady_clock::now();
    auto end_time = std::chrono::seconds(topic_wait_time);
    while (!new_msg_) {
      // Send feedback until the action completes.
      feedback->is_group_detected = false;
      action_server_->publish_feedback(feedback);
      if (std::chrono::steady_clock::now() - start_time > end_time) {
        RCLCPP_WARN(this->get_logger(),
                    "People info topic not published. Stopping.");
        result->success = false;
        action_server_->succeeded_current(result);
        return;
      }
      r.sleep();
    }
    feedback->is_group_detected = true;
    action_server_->publish_feedback(feedback);

    RCLCPP_INFO(this->get_logger(), "Received people info topic");
  }

  // Cancel navigation related tasks
  this->nav_to_pose_client_->async_cancel_all_goals();

  RCLCPP_INFO(this->get_logger(), "Executing goal");
  while (rclcpp::ok()) {
    // Sleep for the remainder of the loop period, to enforce the loop rate
    r.sleep();

    // Check if asked to process another action
    if (action_server_->is_preempt_requested()) {
      RCLCPP_INFO(this->get_logger(), "Preempting the goal pose.");
      goal = action_server_->accept_pending_goal();
      is_goal_active_ = goal->move_to_group;
      new_msg_ = true;
      continue;
    }

    // Check if start tracking is false
    if (!is_goal_active_) {
      RCLCPP_INFO(this->get_logger(),
                  "Exiting since moving towards people group has been "
                  "requested to stop");
      // Cancel navigation related tasks
      this->nav_to_pose_client_->async_cancel_all_goals();
      current_goal_status_ = ActionStatus::CANCELED;
    }

    if (new_msg_ && is_goal_active_) {
      RCLCPP_INFO_STREAM_ONCE(this->get_logger(),
                              "Sending goal to navigate_to_pose server");
      ClientT::Goal goal;

      // Choose only one group
      if (group_poses_.size() > 0) {
        group_pose_ = group_poses_[0];
      } else {
        RCLCPP_WARN(this->get_logger(), "No people group detected. Stopping.");
        current_goal_status_ = ActionStatus::CANCELED;
        continue;
      }
      goal.pose = group_pose_;
      goal.behavior_tree = behavior_tree_path_;

      auto send_goal_options =
          rclcpp_action::Client<ClientT>::SendGoalOptions();
      send_goal_options.result_callback =
          std::bind(&MoveToPeopleActionServer::resultCallback, this,
                    std::placeholders::_1);
      send_goal_options.goal_response_callback =
          std::bind(&MoveToPeopleActionServer::goalResponseCallback, this,
                    std::placeholders::_1);
      future_goal_handle_ =
          this->nav_to_pose_client_->async_send_goal(goal, send_goal_options);
      current_goal_status_ = ActionStatus::PROCESSING;
      new_msg_ = false;
    }
    // Publish threat info marker
    marker_.header.stamp = this->now();
    marker_.pose.position.x = group_pose_.pose.position.x;
    marker_.pose.position.y = group_pose_.pose.position.y;
    marker_pub_->publish(marker_);

    if (group_detected_) {
      current_goal_status_ = ActionStatus::PROCESSING;
    } else {
      // If the actionstatus is processing, there is already a navigation goal
      // running
      if (current_goal_status_ == ActionStatus::PROCESSING) {
        // Wait for navigation to complete
        if (future_goal_handle_.get()->get_status() == GOAL_STATE_SUCCEEDED)
          current_goal_status_ = ActionStatus::SUCCEEDED;
      } else
        // No threat detected and no navigation goal running
        current_goal_status_ = ActionStatus::CANCELED;
    }

    if (current_goal_status_ == ActionStatus::SUCCEEDED) {
      result->success = true;
      action_server_->succeeded_current(result);
      new_msg_ = false;
      return;
    } else if (current_goal_status_ == ActionStatus::CANCELED) {
      if (!group_detected_)
        RCLCPP_INFO(this->get_logger(),
                    "People Group is not Detected. Cancelling.");
      else
        RCLCPP_INFO(this->get_logger(),
                    "Requested to stop moving towards group. Stopping.");

      this->nav_to_pose_client_->async_cancel_all_goals();
      result->success = true;
      action_server_->succeeded_current(result);
      return;
    } else if (current_goal_status_ != ActionStatus::PROCESSING or
               current_goal_status_ == ActionStatus::FAILED) {
      RCLCPP_INFO(this->get_logger(), "Navigation failed. Stopping.");
      result->success = false;
      action_server_->succeeded_current(result);
      return;
    } else if (current_goal_status_ == ActionStatus::PROCESSING) {
      // Feedback send
      feedback->is_group_detected = true;
      feedback->current_pose = group_pose_;
      action_server_->publish_feedback(feedback);
    }
  }
}

void MoveToPeopleActionServer::groupInfoCallback(
    const roast_interfaces::msg::PeopleGroup::SharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Received threat info");
  new_msg_ = true;
  group_detected_ = msg->group_detected;

  // Choose only one group
  if (group_detected_) group_poses_ = msg->group_poses;
}

void MoveToPeopleActionServer::resultCallback(
    const rclcpp_action::ClientGoalHandle<ClientT>::WrappedResult &result) {
  if (result.goal_id != future_goal_handle_.get()->get_goal_id()) {
    RCLCPP_DEBUG(
        get_logger(),
        "Goal IDs do not match for the current goal handle and received result."
        "Ignoring likely due to receiving result for an old goal.");
    return;
  }

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      current_goal_status_ = ActionStatus::SUCCEEDED;
      return;
    case rclcpp_action::ResultCode::ABORTED:
      current_goal_status_ = ActionStatus::CANCELED;
      return;
    case rclcpp_action::ResultCode::CANCELED:
      current_goal_status_ = ActionStatus::SUCCEEDED;
      return;
    default:
      current_goal_status_ = ActionStatus::UNKNOWN;
      return;
  }
}

void MoveToPeopleActionServer::goalResponseCallback(
    const rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr &goal) {
  if (!goal) {
    RCLCPP_ERROR(
        get_logger(),
        "navigate_to_pose action client failed to send goal to server.");
    current_goal_status_ = ActionStatus::FAILED;
  }
}
}  // namespace roast_people_detection

RCLCPP_COMPONENTS_REGISTER_NODE(
    roast_people_detection::MoveToPeopleActionServer)
