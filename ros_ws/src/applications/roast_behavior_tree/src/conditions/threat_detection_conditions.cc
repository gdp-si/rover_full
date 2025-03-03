#include "roast_behavior_tree/conditions/threat_detection_conditions.hpp"

#include <string>

namespace roast_behavior_tree {
IsThreatDetected::IsThreatDetected(const std::string &condition_name,
                                   const BT::NodeConfiguration &conf)
    : BT::ConditionNode(condition_name, conf) {
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
  executor_.add_callback_group(callback_group_,
                               node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  info_sub_ = node_->create_subscription<roast_interfaces::msg::ThreatInfo>(
      "threat_info", rclcpp::SystemDefaultsQoS(),
      std::bind(&IsThreatDetected::threatInfoCallback, this,
                std::placeholders::_1),
      sub_option);

  // Topic
  this->threat_detected_ = false;
}

BT::NodeStatus IsThreatDetected::tick() {
  executor_.spin_some();
  if (threat_detected_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void IsThreatDetected::threatInfoCallback(
    roast_interfaces::msg::ThreatInfo::SharedPtr msg) {
  threat_detected_ = msg->threat_detected;
}

IsNearThreat::IsNearThreat(const std::string &condition_name,
                           const BT::NodeConfiguration &conf)
    : BT::ConditionNode(condition_name, conf) {
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
  executor_.add_callback_group(callback_group_,
                               node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  info_sub_ = node_->create_subscription<roast_interfaces::msg::ThreatInfo>(
      "threat_info", rclcpp::SystemDefaultsQoS(),
      std::bind(&IsNearThreat::threatInfoCallback, this, std::placeholders::_1),
      sub_option);

  // Topic
  this->is_near_threat_ = false;
  this->threat_detected_ = false;
}

BT::NodeStatus IsNearThreat::tick() {
  executor_.spin_some();
  if (threat_detected_ && is_near_threat_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void IsNearThreat::threatInfoCallback(
    roast_interfaces::msg::ThreatInfo::SharedPtr msg) {
  is_near_threat_ = msg->near_safety_perimeter;
  threat_detected_ = msg->threat_detected;
}
}  // namespace roast_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<roast_behavior_tree::IsThreatDetected>(
      "IsThreatDetected");
  factory.registerNodeType<roast_behavior_tree::IsNearThreat>("IsNearThreat");
}
