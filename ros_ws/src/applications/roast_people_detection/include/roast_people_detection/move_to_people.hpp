#ifndef ROAST_PEOPLE_DETECTION__MOVE_TO_PEOPLE_HPP_
#define ROAST_PEOPLE_DETECTION__MOVE_TO_PEOPLE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "roast_interfaces/action/move_to_people.hpp"
#include "roast_interfaces/msg/people_group.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace roast_people_detection {
enum class ActionStatus {
  UNKNOWN = 0,
  PROCESSING = 1,
  FAILED = 2,
  SUCCEEDED = 3,
  CANCELED = 4
};

class MoveToPeopleActionServer : public rclcpp::Node {
 public:
  using ActionT = roast_interfaces::action::MoveToPeople;
  using ClientT = nav2_msgs::action::NavigateToPose;
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;
  using ActionClient = rclcpp_action::Client<ClientT>;
  using Marker = visualization_msgs::msg::Marker;

  MoveToPeopleActionServer(const rclcpp::NodeOptions &options);
  ~MoveToPeopleActionServer();

 protected:
  void move_to_people_group();
  void resultCallback(
      const rclcpp_action::ClientGoalHandle<ClientT>::WrappedResult &result);
  void goalResponseCallback(
      const rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr &goal);

  std::unique_ptr<ActionServer> action_server_;
  ActionClient::SharedPtr nav_to_pose_client_;
  std::shared_future<rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr>
      future_goal_handle_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  ActionStatus current_goal_status_, spin_goal_status_;

  bool stop_on_failure_;
  geometry_msgs::msg::PoseStamped current_robot_pose_;
  int loop_rate_;

  bool is_goal_active_;

 private:
  rclcpp::Subscription<roast_interfaces::msg::PeopleGroup>::SharedPtr
      subscription_;
  bool group_detected_;
  bool new_msg_;
  geometry_msgs::msg::PoseStamped group_pose_;
  std::vector<geometry_msgs::msg::PoseStamped> group_poses_;
  std::string behavior_tree_path_;
  Marker marker_;
  rclcpp::Publisher<Marker>::SharedPtr marker_pub_;

  void groupInfoCallback(
      const roast_interfaces::msg::PeopleGroup::SharedPtr msg);
};

}  // namespace roast_people_detection
#endif  // ROAST_PEOPLE_DETECTION__MOVE_TO_PEOPLE_HPP_
