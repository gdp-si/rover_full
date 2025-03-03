#ifndef ROAST_ACTIONS__SPEAKER_ANNOUNCEMENT_HPP_
#define ROAST_ACTIONS__SPEAKER_ANNOUNCEMENT_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "roast_interfaces/action/speaker_announcement.hpp"
#include "roast_interfaces/srv/alert_mode.hpp"

namespace roast_actions {
class SpeakerAnnouncement : public rclcpp::Node {
 public:
  using ActionT = roast_interfaces::action::SpeakerAnnouncement;
  using ServiceT = roast_interfaces::srv::AlertMode;
  using ActionServer = rclcpp_action::Server<ActionT>;
  using GoalHandleMessage = rclcpp_action::ServerGoalHandle<ActionT>;
  using ServiceClient = rclcpp::Client<ServiceT>;

  SpeakerAnnouncement(const rclcpp::NodeOptions &options);
  ~SpeakerAnnouncement();

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
  ServiceClient::SharedPtr speaker_client_;
};
}  // namespace roast_actions

#endif
