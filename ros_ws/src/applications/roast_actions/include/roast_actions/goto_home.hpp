// This action tries to move the robot to home position. It uses the
// NavigateToPose action to do so. Gets to the pose 1 meter away from the home
// position and then spins to face the home position. Backs up 1 meter and then
// spins to face the home position.
#ifndef ROAST_ACTIONS__GOTO_HOME_HPP_
#define ROAST_ACTIONS__GOTO_HOME_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "roast_interfaces/action/go_to_home.hpp"

namespace roast_actions {
enum class ActionStatus {
  UNKNOWN = 0,
  PROCESSING = 1,
  FAILED = 2,
  SUCCEEDED = 3,
  CANCELED = 4
};

class GoToHomeActionServer : public rclcpp::Node {
 public:
  using ActionT = roast_interfaces::action::GoToHome;
  using NavigateClientT = nav2_msgs::action::NavigateToPose;
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;
  using NavigateClient = rclcpp_action::Client<NavigateClientT>;

  GoToHomeActionServer(const rclcpp::NodeOptions &options);
  ~GoToHomeActionServer();

 protected:
  void goToHome();

  void resultCallback(
      const rclcpp_action::ClientGoalHandle<NavigateClientT>::WrappedResult
          &result);
  void goalResponseCallback(
      const rclcpp_action::ClientGoalHandle<NavigateClientT>::SharedPtr &goal);

  // Our action server
  std::unique_ptr<ActionServer> action_server_;
  NavigateClient::SharedPtr nav_to_pose_client_;
  std::shared_future<
      rclcpp_action::ClientGoalHandle<NavigateClientT>::SharedPtr>
      future_goal_handle_;

 private:
  void navigateToHome(geometry_msgs::msg::PoseStamped goal_pose);
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  bool stop_on_failure_, new_goal_;
  ActionStatus current_goal_status_, navigate_status_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  int loop_rate_;
  double spin_angle_, backup_distance_;

  // Our action server
  ActionT::Result::SharedPtr result_;
  ActionT::Goal::ConstSharedPtr goal_;
};

}  // namespace roast_actions
#endif  // ROAST_ACTIONS__GOTO_HOME_HPP_
