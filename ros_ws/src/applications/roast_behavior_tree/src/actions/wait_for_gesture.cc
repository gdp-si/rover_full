#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <roast_behavior_tree/actions/wait_for_gesture.hpp>
#include <string>

namespace roast_behavior_tree {
WaitForHandGestureNode::WaitForHandGestureNode(
    const std::string &service_node_name, const BT::NodeConfiguration &conf)
    : BtServiceNode<roast_interfaces::srv::WaitForHandGesture>(
          service_node_name, "wait_for_hand_gesture", conf) {
  // Server timeout to allow wait for gesture
  server_timeout_ = std::chrono::milliseconds(
      static_cast<int>(server_timeout_seconds_ * 1000));
}

void WaitForHandGestureNode::on_tick() {
  auto timeout = getInput<double>("timeout").value_or(5.0);

  if (timeout > server_timeout_seconds_) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                "Timeout %f is greater than server timeout %f. Setting timeout "
                "to server timeout.",
                timeout, server_timeout_seconds_);
    timeout = server_timeout_seconds_;
  }
  request_->timeout = timeout;

  // Gesture should be from one of the following:
  // "hi", "fist", "thumb_up"
  std::string gesture = getInput<std::string>("gesture").value_or("hi");
  if (gesture != "hi" && gesture != "fist" && gesture != "thumb_up") {
    std::cerr << "Gesture " << gesture
              << " is not supported. Defaulting to \"hi\"." << std::endl;
    gesture = "hi";
  }
  request_->expected_gesture = gesture;
}
}  // namespace roast_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<roast_behavior_tree::WaitForHandGestureNode>(
      "WaitForHandGesture");
}
