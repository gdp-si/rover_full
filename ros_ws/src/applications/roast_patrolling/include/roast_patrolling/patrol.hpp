#ifndef ROAST_PATROLLING__PATROL_HPP_
#define ROAST_PATROLLING__PATROL_HPP_

#include <fstream>
#include <memory>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "roast_interfaces/action/patrol_action.hpp"
#include "roast_interfaces/srv/get_robot_pose.hpp"

namespace roast_patrolling {
enum class ActionStatus {
  UNKNOWN = 0,
  PROCESSING = 1,
  FAILED = 2,
  SUCCEEDED = 3,
  CANCELED = 4
};

class PatrolAction : public rclcpp::Node {
 public:
  using ActionT = roast_interfaces::action::PatrolAction;
  using ToPoseClientT = nav2_msgs::action::NavigateToPose;
  using ThroughPoseClientT = nav2_msgs::action::NavigateThroughPoses;
  using ServiceT = roast_interfaces::srv::GetRobotPose;
  using MapServiceT = rcl_interfaces::srv::GetParameters;
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;
  using ToPoseActionClient = rclcpp_action::Client<ToPoseClientT>;
  using ThroughPoseActionClient = rclcpp_action::Client<ThroughPoseClientT>;
  using ServiceClient = rclcpp::Client<ServiceT>;
  using MapServiceClient = rclcpp::Client<MapServiceT>;

  PatrolAction(const rclcpp::NodeOptions &options);
  ~PatrolAction();

 protected:
  void patrol();
  void toPoseResultCallback(
      const rclcpp_action::ClientGoalHandle<ToPoseClientT>::WrappedResult
          &result);
  void toPoseGoalResponseCallback(
      const rclcpp_action::ClientGoalHandle<ToPoseClientT>::SharedPtr &goal);
  void throughPosesResultCallback(
      const rclcpp_action::ClientGoalHandle<ThroughPoseClientT>::WrappedResult
          &result);
  void throughPosesGoalResponseCallback(
      const rclcpp_action::ClientGoalHandle<ThroughPoseClientT>::SharedPtr
          &goal);

  // Our action server
  std::unique_ptr<ActionServer> action_server_;
  ToPoseActionClient::SharedPtr nav_to_pose_client_;
  ThroughPoseActionClient::SharedPtr nav_through_poses_client_;
  std::shared_future<rclcpp_action::ClientGoalHandle<ToPoseClientT>::SharedPtr>
      future_to_pose_goal_handle_;
  std::shared_future<
      rclcpp_action::ClientGoalHandle<ThroughPoseClientT>::SharedPtr>
      future_through_poses_goal_handle_;

  ServiceClient::SharedPtr robot_pose_client_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  ActionStatus current_goal_status_;

  bool stop_on_failure_;
  geometry_msgs::msg::PoseStamped current_robot_pose_;
  int loop_rate_;
  uint8_t patrol_type_, patrol_mode_;
  bool start_patrol_;

 private:
  void getWaypointCoordinates(
      const std::string &file_name,
      std::vector<geometry_msgs::msg::PoseStamped> &waypoints);
  std::vector<std::pair<double, double>> readCoordinates(
      const std::string &filename);
  std::vector<geometry_msgs::msg::PoseStamped> filterWaypoints(
      const geometry_msgs::msg::PoseStamped &current_robot_pose);
  inline size_t getIndexOfMinValue(const std::vector<double> &distances);
  inline bool cancelNavigationToPose();
  inline bool cancelNavigationThroughPoses();

  void performPatrolWithoutPause(ActionT::Goal::ConstSharedPtr goal,
                                 ActionT::Feedback::SharedPtr feedback,
                                 ActionT::Result::SharedPtr result);
  void performPatrolWithPause(ActionT::Goal::ConstSharedPtr goal,
                              ActionT::Feedback::SharedPtr feedback,
                              ActionT::Result::SharedPtr result);
  void currentRobotPose();

  std::string file_name_;
  std::string behavior_tree_path_, with_pause_behavior_tree_path_,
      without_pause_behavior_tree_path_;
  std::string map_name_;
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  std::string package_share_directory;
};
}  // namespace roast_patrolling

#endif  // ROAST_PATROLLING__PATROL_HPP_
