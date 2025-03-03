#include "roast_behavior_tree/conditions/is_group_detected.hpp"

#include <string>

namespace roast_behavior_tree {
IsGroupDetected::IsGroupDetected(const std::string &condition_name,
                                 const BT::NodeConfiguration &conf)
    : BT::ConditionNode(condition_name, conf) {
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
  executor_.add_callback_group(callback_group_,
                               node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  info_sub_ = node_->create_subscription<roast_interfaces::msg::PeopleGroup>(
      "people_info", rclcpp::SystemDefaultsQoS(),
      std::bind(&IsGroupDetected::groupInfoCallback, this,
                std::placeholders::_1),
      sub_option);

  // Topic
  this->group_detected_ = false;
}

BT::NodeStatus IsGroupDetected::tick() {
  executor_.spin_some();
  if (group_detected_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void IsGroupDetected::groupInfoCallback(
    roast_interfaces::msg::PeopleGroup::SharedPtr msg) {
  group_detected_ = msg->group_detected;
}

}  // namespace roast_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<roast_behavior_tree::IsGroupDetected>(
      "IsGroupDetected");
}
