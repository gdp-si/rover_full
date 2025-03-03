#ifndef ROAST_ACTIONS__MOVE_FORWARD_HPP_
#define ROAST_ACTIONS__MOVE_FORWARD_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "roast_interfaces/action/move_forward.hpp"
#include "roast_interfaces/srv/get_robot_pose.hpp"

namespace roast_actions {

enum class ActionStatus {
  UNKNOWN = 0,
  PROCESSING = 1,
  FAILED = 2,
  SUCCEEDED = 3,
  CANCELED = 4
};

class MoveForward : public rclcpp::Node {
 public:
  using ActionT = roast_interfaces::action::MoveForward;
  using ClientT = nav2_msgs::action::NavigateToPose;
  using ServiceT = roast_interfaces::srv::GetRobotPose;
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;
  using ActionClient = rclcpp_action::Client<ClientT>;
  using ServiceClient = rclcpp::Client<ServiceT>;

  MoveForward(const rclcpp::NodeOptions &options);
  ~MoveForward();

 protected:
  void moveForward();

  void resultCallback(
      const rclcpp_action::ClientGoalHandle<ClientT>::WrappedResult &result);
  void goalResponseCallback(
      const rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr &goal);

  // Our action server
  std::unique_ptr<ActionServer> action_server_;
  ActionClient::SharedPtr nav_to_pose_client_;
  ServiceClient::SharedPtr robot_pose_client_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::shared_future<rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr>
      future_goal_handle_;
  bool stop_on_failure_;
  ActionStatus current_goal_status_;
  geometry_msgs::msg::PoseStamped current_robot_pose_;
  int loop_rate_;
  std::vector<int> failed_ids_;
  double distance_, speed_;
};
}  // namespace roast_actions

#endif
