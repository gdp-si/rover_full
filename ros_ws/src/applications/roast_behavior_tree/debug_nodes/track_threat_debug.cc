// Debug Action Server node for TrackThreat.action
// To test the node, run the following command:
// ros2 action send_goal /track_threat roast_interfaces/action/TrackTarget
// "{"stamp": {"sec": 0, "nanosec": 0}, "message": "Hello", "message_level": 0}"

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "roast_interfaces/action/track_target.hpp"

namespace roast_behavior_tree {
class TrackThreatDebug : public rclcpp::Node {
 public:
  using Message = roast_interfaces::action::TrackTarget;
  using GoalHandleMessage = rclcpp_action::ServerGoalHandle<Message>;

  explicit TrackThreatDebug(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("track_threat_debug", options) {
    using namespace std::placeholders;
    this->action_server_ = rclcpp_action::create_server<Message>(
        this, "track_threat",
        std::bind(&TrackThreatDebug::handle_goal, this, _1, _2),
        std::bind(&TrackThreatDebug::handle_cancel, this, _1),
        std::bind(&TrackThreatDebug::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "Track Threat Debug Node is initialized");
  }

 private:
  rclcpp_action::Server<Message>::SharedPtr action_server_;
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const Message::Goal> goal) {
    if (goal->start_tracking)
      RCLCPP_INFO(this->get_logger(), "Received new goal request");
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
    std::thread{std::bind(&TrackThreatDebug::execute, this, _1), goal_handle}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleMessage> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    auto feedback = std::make_shared<Message::Feedback>();
    auto &goal = goal_handle->get_goal();
    auto result = std::make_shared<Message::Result>();

    rclcpp::Rate loop_rate(1);
    uint counter = 0;
    while (goal->start_tracking) {
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

      feedback->current_pose = geometry_msgs::msg::PoseStamped();
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publishing feedback:  Tracking Threat");
      loop_rate.sleep();
      counter++;

      if (counter >= 10) break;
    }

    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
};
}  // namespace roast_behavior_tree

RCLCPP_COMPONENTS_REGISTER_NODE(roast_behavior_tree::TrackThreatDebug)
