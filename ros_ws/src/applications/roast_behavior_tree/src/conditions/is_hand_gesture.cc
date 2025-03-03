#include "roast_behavior_tree/conditions/is_hand_gesture.hpp"

#include <string>

namespace roast_behavior_tree {
IsHandGestureDetected::IsHandGestureDetected(const std::string &condition_name,
                                             const BT::NodeConfiguration &conf)
    : BT::ConditionNode(condition_name, conf) {
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
  executor_.add_callback_group(callback_group_,
                               node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  info_sub_ = node_->create_subscription<roast_interfaces::msg::HandGesture>(
      "hand_gesture", rclcpp::SystemDefaultsQoS(),
      std::bind(&IsHandGestureDetected::HandGestureCallback, this,
                std::placeholders::_1),
      sub_option);

  // Topic
  this->is_hand_gesture_ = false;
}

BT::NodeStatus IsHandGestureDetected::tick() {
  executor_.spin_some();
  if (is_hand_gesture_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void IsHandGestureDetected::HandGestureCallback(
    roast_interfaces::msg::HandGesture::SharedPtr msg) {
  is_hand_gesture_ = msg->is_gesture;
  gesture_ = msg->gesture;
  setOutput("gesture", std::to_string(gesture_));
}

}  // namespace roast_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<roast_behavior_tree::IsHandGestureDetected>(
      "IsHandGestureDetected");
}
