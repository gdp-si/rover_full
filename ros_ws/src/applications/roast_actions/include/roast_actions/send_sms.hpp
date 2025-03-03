#ifndef ROAST_ACTIONS__SEND_SMS_HPP_
#define ROAST_ACTIONS__SEND_SMS_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "roast_interfaces/action/send_sms_message.hpp"
#include "roast_interfaces/srv/alert_mode.hpp"

namespace roast_actions {
class SendSMS : public rclcpp::Node {
 public:
  using ActionT = roast_interfaces::action::SendSmsMessage;
  using ServiceT = roast_interfaces::srv::AlertMode;
  using ActionServer = rclcpp_action::Server<ActionT>;
  using GoalHandleMessage = rclcpp_action::ServerGoalHandle<ActionT>;
  using ServiceClient = rclcpp::Client<ServiceT>;

  SendSMS(const rclcpp::NodeOptions &options);
  ~SendSMS();

 protected:
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const ActionT::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleMessage> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleMessage> goal_handle);
  void execute(const std::shared_ptr<GoalHandleMessage> goal_handle);

  // Our action server
  ActionServer::SharedPtr action_server_;

  // SMS service client
  ServiceClient::SharedPtr sms_client_;
};
}  // namespace roast_actions

#endif
