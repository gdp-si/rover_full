#include "roast_behavior_tree/actions/recovery_actions.hpp"

#include <memory>
#include <string>

namespace roast_behavior_tree {
SpinAction::SpinAction(const std::string &xml_tag_name,
                       const std::string &action_name,
                       const BT::NodeConfiguration &config)
    : BtActionNode<nav2_msgs::action::Spin>(xml_tag_name, action_name, config) {
}

void SpinAction::on_tick() {
  auto result = getInput<double>("spin_angle");
  if (!result) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input [spin_angle]");
    throw BT::RuntimeError("Missing required input", result.error());
  }
  spin_angle_ = result.value();
  goal_.target_yaw = spin_angle_;
}

BackUpAction::BackUpAction(const std::string &xml_tag_name,
                           const std::string &action_name,
                           const BT::NodeConfiguration &config)
    : BtActionNode<nav2_msgs::action::BackUp>(xml_tag_name, action_name,
                                              config) {}

void BackUpAction::on_tick() {
  auto result = getInput<double>("backup_distance");
  if (!result) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Missing required input [backup_distance]");
    throw BT::RuntimeError("Missing required input", result.error());
  } else {
    backup_distance_ = result.value();
  }
  result = getInput<double>("speed");
  if (!result) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input [speed]");
    throw BT::RuntimeError("Missing required input", result.error());
  } else {
    speed_ = result.value();
  }

  goal_.target.x = backup_distance_;
  goal_.speed = speed_;
}

WaitAction::WaitAction(const std::string &xml_tag_name,
                       const std::string &action_name,
                       const BT::NodeConfiguration &config)
    : BtActionNode<nav2_msgs::action::Wait>(xml_tag_name, action_name, config) {
}

void WaitAction::on_tick() {
  auto result = getInput<double>("wait_duration");
  if (!result) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input [wait_duration]");
    throw BT::RuntimeError("Missing required input", result.error());
  } else {
    wait_duration_ = result.value();
  }
  goal_.time = rclcpp::Duration::from_seconds(wait_duration_);
}
}  // namespace roast_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
template <typename NodeType>
void registerBTNode(const std::string &action_name,
                    const std::string &action_topic,
                    BT::BehaviorTreeFactory &factory) {
  BT::NodeBuilder builder = [&, action_topic](
                                const std::string &name,
                                const BT::NodeConfiguration &config) {
    return std::make_unique<NodeType>(name, action_topic, config);
  };
  factory.registerBuilder<NodeType>(action_name, builder);
  return;
};

BT_REGISTER_NODES(factory) {
  registerBTNode<roast_behavior_tree::SpinAction>("SpinAction", "spin",
                                                  factory);
  registerBTNode<roast_behavior_tree::BackUpAction>("BackUpAction", "backup",
                                                    factory);
  registerBTNode<roast_behavior_tree::WaitAction>("WaitAction", "wait",
                                                  factory);
}
