#ifndef ROAST_THREAT_TRACKING__THREAT_TRACKING_HPP_
#define ROAST_THREAT_TRACKING__THREAT_TRACKING_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "roast_interfaces/action/track_target.hpp"
#include "roast_interfaces/msg/threat_info.hpp"
#include "visualization_msgs/msg/marker.hpp"
namespace roast_threat_tracking {
enum class ActionStatus {
  UNKNOWN = 0,
  PROCESSING = 1,
  FAILED = 2,
  SUCCEEDED = 3,
  CANCELED = 4
};

class ThreatTracker : public rclcpp::Node {
 public:
  using ActionT = roast_interfaces::action::TrackTarget;
  using ClientT = nav2_msgs::action::NavigateToPose;
  using SpinClientT = nav2_msgs::action::Spin;
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;
  using ActionClient = rclcpp_action::Client<ClientT>;
  using SpinActionClient = rclcpp_action::Client<SpinClientT>;
  using Marker = visualization_msgs::msg::Marker;

  ThreatTracker(const rclcpp::NodeOptions &options);
  ~ThreatTracker();

 protected:
  void threatTracker();
  void spinToTarget();
  void resultCallback(
      const rclcpp_action::ClientGoalHandle<ClientT>::WrappedResult &result);
  void goalResponseCallback(
      const rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr &goal);
  void spinResultCallback(
      const rclcpp_action::ClientGoalHandle<SpinClientT>::WrappedResult
          &result);
  void spinGoalResponseCallback(
      const rclcpp_action::ClientGoalHandle<SpinClientT>::SharedPtr &goal);

  std::unique_ptr<ActionServer> action_server_;
  ActionClient::SharedPtr nav_to_pose_client_;
  SpinActionClient::SharedPtr spin_client_;
  std::shared_future<rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr>
      future_goal_handle_;
  std::shared_future<SpinActionClient::GoalHandle::SharedPtr>
      future_spin_goal_handle_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  ActionStatus current_goal_status_, spin_goal_status_;

  bool stop_on_failure_;
  geometry_msgs::msg::PoseStamped current_robot_pose_;
  int loop_rate_;

  bool start_tracking_, turn_to_target_;
  double threat_yaw_;

 private:
  rclcpp::Subscription<roast_interfaces::msg::ThreatInfo>::SharedPtr
      subscription_;
  bool threat_detected_, near_threat_;
  bool new_msg_;
  geometry_msgs::msg::PoseStamped threat_pose_;
  std::string behavior_tree_path_;
  Marker marker_;
  rclcpp::Publisher<Marker>::SharedPtr marker_pub_;

  void threatInfoCallback(
      const roast_interfaces::msg::ThreatInfo::SharedPtr msg);
};

}  // namespace roast_threat_tracking
#endif  // ROAST_THREAT_TRACKING__THREAT_TRACKING_HPP_
