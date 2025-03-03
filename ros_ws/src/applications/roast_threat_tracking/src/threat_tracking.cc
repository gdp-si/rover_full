// To test the node, run the following command:
// ros2 action send_goal /track_threat roast_interfaces/action/TrackTarget
// "{'start_tracking': 'true', 'turn_to_target': 'true'}"

#include "roast_threat_tracking/threat_tracking.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp_components/register_node_macro.hpp"

namespace roast_threat_tracking {
ThreatTracker::ThreatTracker(const rclcpp::NodeOptions &options)
    : Node("threat_tracking_action_server", options) {
  using namespace std::placeholders;
  using namespace std::chrono_literals;

  declare_parameter("stop_on_failure", true);
  declare_parameter("loop_rate", 20);

  stop_on_failure_ = get_parameter("stop_on_failure").as_bool();
  loop_rate_ = get_parameter("loop_rate").as_int();
  std::string package_share_directory =
      ament_index_cpp::get_package_share_directory("roast_threat_tracking");
  behavior_tree_path_ =
      package_share_directory + "/behavior_trees/threat_tracker.xml";
  threat_detected_ = false;

  nav_to_pose_client_ = rclcpp_action::create_client<ClientT>(
      get_node_base_interface(), get_node_graph_interface(),
      get_node_logging_interface(), get_node_waitables_interface(),
      "navigate_to_pose", callback_group_);

  action_server_ = std::make_unique<ActionServer>(
      get_node_base_interface(), get_node_clock_interface(),
      get_node_logging_interface(), get_node_waitables_interface(),
      "track_threat", std::bind(&ThreatTracker::threatTracker, this));

  subscription_ = this->create_subscription<roast_interfaces::msg::ThreatInfo>(
      "threat_info", 15,
      std::bind(&ThreatTracker::threatInfoCallback, this,
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

  RCLCPP_INFO(this->get_logger(), "Threat Tracking Action Server is active");

  marker_pub_ = this->create_publisher<Marker>("threat_info/marker", 1);
  marker_.header.frame_id = "map";
  marker_.header.stamp = this->now();
  marker_.ns = "threat_info";
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
  marker_.color.r = 1.0;
  marker_.color.g = 1.0;
  marker_.color.b = 0.0;
}

ThreatTracker::~ThreatTracker() {}

void ThreatTracker::threatTracker() {
  auto goal = action_server_->get_current_goal();
  auto feedback = std::make_shared<ActionT::Feedback>();
  auto result = std::make_shared<ActionT::Result>();

  start_tracking_ = goal->start_tracking;
  turn_to_target_ = goal->turn_to_target;
  auto topic_wait_time = goal->topic_timeout;
  new_msg_ = false;
  threat_detected_ = false;
  near_threat_ = false;

  // Check if request is valid
  if (!action_server_ || !action_server_->is_server_active()) {
    RCLCPP_WARN(this->get_logger(), "Action Server inactive. Stopping.");
    result->success = false;
    action_server_->succeeded_current(result);
    return;
  }

  if (!start_tracking_) {
    RCLCPP_INFO(this->get_logger(),
                "Exiting since threat tracking has been requested to stop");
    // Cancel navigation related tasks
    this->nav_to_pose_client_->async_cancel_all_goals();
    result->success = true;
    action_server_->succeeded_current(result);
    return;
  }

  rclcpp::WallRate r(loop_rate_);

  if (!new_msg_) {
    RCLCPP_INFO(this->get_logger(),
                "Waiting for threat info topic to be published");
    auto start_time = std::chrono::steady_clock::now();
    auto end_time = std::chrono::seconds(topic_wait_time);
    while (!new_msg_) {
      // Send feedback until the action completes.
      feedback->is_tracking = true;
      action_server_->publish_feedback(feedback);
      if (std::chrono::steady_clock::now() - start_time > end_time) {
        RCLCPP_WARN(this->get_logger(),
                    "Threat info topic not published. Stopping.");
        result->success = false;
        action_server_->succeeded_current(result);
        return;
      }
      r.sleep();
    }
    RCLCPP_INFO(this->get_logger(), "Received threat info topic");
  }

  // Check if the threat is already near the safety perimeter
  if (near_threat_) {
    RCLCPP_INFO(this->get_logger(), "Robot is near threat. Finishing up.");
    result->success = true;
    action_server_->succeeded_current(result);
    return;
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
      start_tracking_ = goal->start_tracking;
      new_msg_ = true;
      continue;
    }

    // Check if start tracking is false
    if (!start_tracking_) {
      RCLCPP_INFO(this->get_logger(),
                  "Exiting since threat tracking has been requested to stop");
      // Cancel navigation related tasks
      this->nav_to_pose_client_->async_cancel_all_goals();
      current_goal_status_ = ActionStatus::CANCELED;
    }

    if (new_msg_ && threat_detected_ && !near_threat_) {
      RCLCPP_INFO_STREAM_ONCE(this->get_logger(),
                              "Sending goal to navigate_to_pose server");
      ClientT::Goal goal;
      goal.pose = threat_pose_;
      goal.behavior_tree = behavior_tree_path_;

      auto send_goal_options =
          rclcpp_action::Client<ClientT>::SendGoalOptions();
      send_goal_options.result_callback = std::bind(
          &ThreatTracker::resultCallback, this, std::placeholders::_1);
      send_goal_options.goal_response_callback = std::bind(
          &ThreatTracker::goalResponseCallback, this, std::placeholders::_1);
      future_goal_handle_ =
          this->nav_to_pose_client_->async_send_goal(goal, send_goal_options);
      current_goal_status_ = ActionStatus::PROCESSING;
      new_msg_ = false;
    }
    // Publish threat info marker
    marker_.header.stamp = this->now();
    marker_.pose.position.x = threat_pose_.pose.position.x;
    marker_.pose.position.y = threat_pose_.pose.position.y;
    marker_pub_->publish(marker_);

    if (threat_detected_) {
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
    if (near_threat_) {
      RCLCPP_INFO(this->get_logger(), "Robot is near threat. Finishing up.");
      current_goal_status_ = ActionStatus::SUCCEEDED;
    }

    if (current_goal_status_ == ActionStatus::SUCCEEDED) {
      if (turn_to_target_) {
        this->spinToTarget();
      }
      result->success = true;
      action_server_->succeeded_current(result);
      new_msg_ = false;
      return;
    } else if (current_goal_status_ == ActionStatus::CANCELED) {
      if (!threat_detected_)
        RCLCPP_INFO(
            this->get_logger(),
            "Threat is not Detected. Stopping Navigation and Tracking.");
      else
        RCLCPP_INFO(this->get_logger(),
                    "Requested to stop tracking. Stopping.");
      // TODO: Uncomment once the detection model is improved
      // this->nav_to_pose_client_->async_cancel_all_goals();
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
      feedback->is_tracking = true;
      feedback->current_pose = threat_pose_;
      action_server_->publish_feedback(feedback);
    }
  }
}

void ThreatTracker::spinToTarget() {
  if (!turn_to_target_) {
    RCLCPP_INFO(this->get_logger(),
                "Exiting spin action since turn to target is false");
    return;
  }

  if (!threat_detected_) {
    RCLCPP_INFO(this->get_logger(),
                "Exiting spin action since threat is no longer detected");
    return;
  }
  RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Sending goal to spin server");
  SpinClientT::Goal goal;
  goal.target_yaw = threat_yaw_;

  auto send_goal_options =
      rclcpp_action::Client<SpinClientT>::SendGoalOptions();
  send_goal_options.result_callback = std::bind(
      &ThreatTracker::spinResultCallback, this, std::placeholders::_1);
  send_goal_options.goal_response_callback = std::bind(
      &ThreatTracker::spinGoalResponseCallback, this, std::placeholders::_1);
  future_spin_goal_handle_ =
      this->spin_client_->async_send_goal(goal, send_goal_options);
  spin_goal_status_ = ActionStatus::PROCESSING;

  while (rclcpp::ok()) {
    if (spin_goal_status_ == ActionStatus::SUCCEEDED ||
        spin_goal_status_ == ActionStatus::CANCELED) {
      return;
    }
  }
}

void ThreatTracker::threatInfoCallback(
    const roast_interfaces::msg::ThreatInfo::SharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Received threat info");
  new_msg_ = true;
  threat_detected_ = msg->threat_detected;
  near_threat_ = msg->near_safety_perimeter;
  threat_pose_ = msg->threat_pose;
  threat_yaw_ = msg->threat_yaw;
}

void ThreatTracker::resultCallback(
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

void ThreatTracker::goalResponseCallback(
    const rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr &goal) {
  if (!goal) {
    RCLCPP_ERROR(
        get_logger(),
        "navigate_to_pose action client failed to send goal to server.");
    current_goal_status_ = ActionStatus::FAILED;
  }
}

void ThreatTracker::spinResultCallback(
    const rclcpp_action::ClientGoalHandle<SpinClientT>::WrappedResult &result) {
  if (result.goal_id != future_spin_goal_handle_.get()->get_goal_id()) {
    RCLCPP_DEBUG(
        get_logger(),
        "Goal IDs do not match for the current goal handle and received result."
        "Ignoring likely due to receiving result for an old goal.");
    return;
  }

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      spin_goal_status_ = ActionStatus::SUCCEEDED;
      return;
    case rclcpp_action::ResultCode::ABORTED:
      spin_goal_status_ = ActionStatus::FAILED;
      return;
    case rclcpp_action::ResultCode::CANCELED:
      spin_goal_status_ = ActionStatus::CANCELED;
      return;
    default:
      spin_goal_status_ = ActionStatus::UNKNOWN;
      return;
  }
}

void ThreatTracker::spinGoalResponseCallback(
    const rclcpp_action::ClientGoalHandle<SpinClientT>::SharedPtr &goal) {
  if (!goal) {
    RCLCPP_ERROR(get_logger(),
                 "spin action client failed to send goal to server.");
    spin_goal_status_ = ActionStatus::FAILED;
  }
}
}  // namespace roast_threat_tracking

RCLCPP_COMPONENTS_REGISTER_NODE(roast_threat_tracking::ThreatTracker)
