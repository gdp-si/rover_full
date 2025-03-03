// To test the node, run the following command:
// ros2 action send_goal /move_forward roast_interfaces/action/MoveForward
// "{'distance': {'data': 1.0}, 'speed': {'data': 1.0}}"

#include "roast_actions/move_forward.hpp"

#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp_components/register_node_macro.hpp"

namespace roast_actions {

MoveForward::MoveForward(const rclcpp::NodeOptions &options)
    : Node("move_forward_action_server", options) {
  using namespace std::placeholders;
  using namespace std::chrono_literals;

  declare_parameter("stop_on_failure", true);
  declare_parameter("loop_rate", 20);

  // Configure
  stop_on_failure_ = get_parameter("stop_on_failure").as_bool();
  loop_rate_ = get_parameter("loop_rate").as_int();

  nav_to_pose_client_ = rclcpp_action::create_client<ClientT>(
      get_node_base_interface(), get_node_graph_interface(),
      get_node_logging_interface(), get_node_waitables_interface(),
      "navigate_to_pose", callback_group_);

  action_server_ = std::make_unique<ActionServer>(
      get_node_base_interface(), get_node_clock_interface(),
      get_node_logging_interface(), get_node_waitables_interface(),
      "move_forward", std::bind(&MoveForward::moveForward, this));

  robot_pose_client_ = this->create_client<ServiceT>("get_robot_pose");

  // Activate
  action_server_->activate();
}

MoveForward::~MoveForward() { action_server_->terminate_all(); }

void MoveForward::moveForward() {
  auto goal = action_server_->get_current_goal();
  auto feedback = std::make_shared<ActionT::Feedback>();
  auto result = std::make_shared<ActionT::Result>();

  distance_ = goal->distance.data;
  speed_ = goal->speed.data;

  // Check if request is valid
  if (!action_server_ || !action_server_->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server inactive. Stopping.");
    return;
  }

  if (goal->distance.data <= 0) {
    action_server_->succeeded_current(result);
    return;
  }

  rclcpp::WallRate r(loop_rate_);

  // Cancel navigation related tasks
  this->nav_to_pose_client_->async_cancel_all_goals();

  bool new_goal = true;
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  while (rclcpp::ok()) {
    auto pose_request = std::make_shared<ServiceT::Request>();
    pose_request->get_current_pose = true;
    auto pose_result =
        this->robot_pose_client_->async_send_request(pose_request);
    try {
      // Spin Until complete fails since executing in a separate thread
      auto response = pose_result.get();
      if (response->robot_pose.header.frame_id != "") {
        RCLCPP_INFO_ONCE(this->get_logger(), "Acquired current robot pose");
        current_robot_pose_ = response->robot_pose;
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Failed to get robot's current pose. Exiting current action request");
      return;
    }

    // Check if asked to stop processing action
    if (action_server_->is_cancel_requested()) {
      RCLCPP_INFO(this->get_logger(), "Cancelling current action request");
      auto cancel_future = nav_to_pose_client_->async_cancel_all_goals();
      try {
        // Spin Until complete fails since executing in a separate thread
        auto response = cancel_future.get();
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(),
                     "Unable to cancel current action request");
      }
      // for result callback processing
      new_goal = true;
      current_goal_status_ = ActionStatus::CANCELED;
      return;
    }

    // Check if asked to process another action
    if (action_server_->is_preempt_requested()) {
      RCLCPP_INFO(get_logger(), "Preempting the goal pose.");
      goal = action_server_->accept_pending_goal();
      distance_ = goal->distance.data;
      speed_ = goal->speed.data;
      new_goal = true;
    }

    if (new_goal) {
      RCLCPP_INFO(this->get_logger(),
                  "Sending goal to navigate_to_pose server");
      ClientT::Goal goal;
      goal.pose = current_robot_pose_;
      // TODO: Move Forward Distance
      goal.pose.pose.position.x += distance_;
      auto send_goal_options =
          rclcpp_action::Client<ClientT>::SendGoalOptions();
      send_goal_options.result_callback =
          std::bind(&MoveForward::resultCallback, this, std::placeholders::_1);
      send_goal_options.goal_response_callback = std::bind(
          &MoveForward::goalResponseCallback, this, std::placeholders::_1);
      future_goal_handle_ =
          nav_to_pose_client_->async_send_goal(goal, send_goal_options);
      current_goal_status_ = ActionStatus::PROCESSING;
      new_goal = false;
    }

    if (current_goal_status_ == ActionStatus::SUCCEEDED ||
        current_goal_status_ == ActionStatus::CANCELED) {
      result->success = true;
      action_server_->succeeded_current(result);
      return;
    } else if (current_goal_status_ != ActionStatus::PROCESSING) {
      result->success = false;
      action_server_->succeeded_current(result);
      return;
    } else {
      feedback->current_pose = current_robot_pose_;
      action_server_->publish_feedback(feedback);
    }
  }
}

void MoveForward::resultCallback(
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
      current_goal_status_ = ActionStatus::FAILED;
      return;
    case rclcpp_action::ResultCode::CANCELED:
      current_goal_status_ = ActionStatus::FAILED;
      return;
    default:
      current_goal_status_ = ActionStatus::UNKNOWN;
      return;
  }
}

void MoveForward::goalResponseCallback(
    const rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr &goal) {
  if (!goal) {
    RCLCPP_ERROR(
        get_logger(),
        "navigate_to_pose action client failed to send goal to server.");
    current_goal_status_ = ActionStatus::FAILED;
  }
}

}  // namespace roast_actions

RCLCPP_COMPONENTS_REGISTER_NODE(roast_actions::MoveForward)
