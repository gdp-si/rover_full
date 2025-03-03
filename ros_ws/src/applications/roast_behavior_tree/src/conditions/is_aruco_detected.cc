#include "roast_behavior_tree/conditions/is_aruco_detected.hpp"

#include <string>

namespace roast_behavior_tree {
IsArucoDetected::IsArucoDetected(const std::string &condition_name,
                                 const BT::NodeConfiguration &conf)
    : BT::ConditionNode(condition_name, conf) {
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
  executor_.add_callback_group(callback_group_,
                               node_->get_node_base_interface());

  detect_aruco_client_ =
      node_->create_client<roast_interfaces::srv::DetectAruco>("detect_aruco");

  // Wait for service
  while (!detect_aruco_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
  }

  // Variables
  this->aruco_detected_ = false;

  // Get input
  image_topic_ = getInput<std::string>("image_topic").value();
  timeout_ =
      static_cast<uint8_t>(std::stoi(getInput<std::string>("timeout").value()));
}

BT::NodeStatus IsArucoDetected::tick() {
  // Call service
  auto request =
      std::make_shared<roast_interfaces::srv::DetectAruco::Request>();
  request->image_topic = image_topic_;
  request->timeout = timeout_;

  auto future = detect_aruco_client_->async_send_request(request);
  executor_.spin_until_future_complete(future);

  aruco_detected_ = future.get()->success;

  if (aruco_detected_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace roast_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<roast_behavior_tree::IsArucoDetected>(
      "IsArucoDetected");
}
