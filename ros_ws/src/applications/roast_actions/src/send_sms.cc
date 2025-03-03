// To test the node, run the following command:
// ros2 action send_goal /send_sms_message
// roast_interfaces/action/SendSmsMessage
// "{"stamp": {"sec": 0, "nanosec": 0}, "message": "Threat detected",
// "message_level": 1, "frequency": 5}"

#include "roast_actions/send_sms.hpp"

#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp_components/register_node_macro.hpp"

namespace roast_actions {

SendSMS::SendSMS(const rclcpp::NodeOptions &options)
    : Node("send_sms_action_server", options) {
  using namespace std::placeholders;
  using namespace std::chrono_literals;

  this->action_server_ = rclcpp_action::create_server<ActionT>(
      this, "send_sms_message", std::bind(&SendSMS::handle_goal, this, _1, _2),
      std::bind(&SendSMS::handle_cancel, this, _1),
      std::bind(&SendSMS::handle_accepted, this, _1));

  this->sms_client_ =
      this->create_client<roast_interfaces::srv::AlertMode>("set_alert_mode");
}

SendSMS::~SendSMS() {}

rclcpp_action::GoalResponse SendSMS::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const SendSMS::ActionT::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received goal request with order %s",
              goal->message.c_str());
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SendSMS::handle_cancel(
    const std::shared_ptr<GoalHandleMessage> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel Send SMS");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SendSMS::handle_accepted(
    const std::shared_ptr<GoalHandleMessage> goal_handle) {
  using namespace std::placeholders;
  std::thread{std::bind(&SendSMS::execute, this, _1), goal_handle}.detach();
}

void SendSMS::execute(const std::shared_ptr<GoalHandleMessage> goal_handle) {
  using namespace std::chrono_literals;
  RCLCPP_INFO(this->get_logger(), "Executing Send SMS");
  auto feedback = std::make_shared<ActionT::Feedback>();
  auto &goal = goal_handle->get_goal();
  auto result = std::make_shared<ActionT::Result>();

  int frequency = goal->frequency;

  rclcpp::Rate loop_rate(1);
  for (int i = 1; (i <= frequency) && rclcpp::ok(); i++) {
    if (goal_handle->is_canceling()) {
      result->success = false;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "SMS canceled");
      return;
    }

    // Sending sms through services
    auto srv_request =
        std::make_shared<roast_interfaces::srv::AlertMode::Request>();
    srv_request->sms = true;
    srv_request->message = goal->message;

    while (!sms_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO(this->get_logger(),
                  "service not available, waiting again...");
    }

    auto srv_result = sms_client_->async_send_request(srv_request);
    // Wait for the result.
    try {
      // Spin Until complete fails since executing in a separate thread
      auto response = srv_result.get();
      if (response->status) {
        RCLCPP_INFO_ONCE(this->get_logger(), "SMS sent");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service ");
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "GSM service not avialable");
      return;
    }

    feedback->status = "Sending SMS";
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "Publishing feedback %d: %s", i,
                feedback->status.c_str());
    loop_rate.sleep();
  }

  result->success = true;
  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_logger(), "SMS sent");
}

}  // namespace roast_actions

RCLCPP_COMPONENTS_REGISTER_NODE(roast_actions::SendSMS)
