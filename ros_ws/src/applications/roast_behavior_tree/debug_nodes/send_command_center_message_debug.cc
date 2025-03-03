// Debug Action Server node for SendCommandCenterMessage.action
// To test the node, run the following command:
// ros2 action send_goal /send_command_center_message
// roast_interfaces/action/SendCommandCenterMessage
// "{"stamp": {"sec": 0, "nanosec": 0}, "message": "Hello", "message_level": 0}"

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "roast_interfaces/action/send_command_center_message.hpp"

namespace roast_behavior_tree {
class SendCommandCenterMessageDebug : public rclcpp::Node {
 public:
  using Message = roast_interfaces::action::SendCommandCenterMessage;
  using GoalHandleMessage = rclcpp_action::ServerGoalHandle<Message>;

  explicit SendCommandCenterMessageDebug(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("send_command_center_message_debug", options) {
    using namespace std::placeholders;
    this->action_server_ = rclcpp_action::create_server<Message>(
        this, "send_command_center_message",
        std::bind(&SendCommandCenterMessageDebug::handle_goal, this, _1, _2),
        std::bind(&SendCommandCenterMessageDebug::handle_cancel, this, _1),
        std::bind(&SendCommandCenterMessageDebug::handle_accepted, this, _1));
  }

 private:
  rclcpp_action::Server<Message>::SharedPtr action_server_;
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const Message::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %s",
                goal->message.c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleMessage> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMessage> goal_handle) {
    using namespace std::placeholders;
    std::thread{std::bind(&SendCommandCenterMessageDebug::execute, this, _1),
                goal_handle}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleMessage> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    auto feedback = std::make_shared<Message::Feedback>();
    auto &goal = goal_handle->get_goal();
    auto result = std::make_shared<Message::Result>();

    rclcpp::Rate loop_rate(1);
    for (uint i = 1; i <= goal->message.size(); i++) {
      if (!rclcpp::ok()) {
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      feedback->error = goal->message.substr(0, i);
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publishing feedback: %s",
                  feedback->error.c_str());
      loop_rate.sleep();
    }

    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
};
}  // namespace roast_behavior_tree

RCLCPP_COMPONENTS_REGISTER_NODE(
    roast_behavior_tree::SendCommandCenterMessageDebug)
