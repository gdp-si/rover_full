#include "roast_behavior_tree/actions/move_forward.hpp"

#include <memory>
#include <string>

namespace roast_behavior_tree {
MoveForwardAction::MoveForwardAction(const std::string &xml_tag_name,
                                     const std::string &action_name,
                                     const BT::NodeConfiguration &config)
    : BtActionNode<Action>(xml_tag_name, action_name, config) {}

void MoveForwardAction::on_tick() {
  getInput<double>("distance", distance_);
  getInput<double>("speed", speed_);

  if (!distance_) {
    throw BT::RuntimeError("Missing required Input: distance");
  }

  if (!speed_) {
    throw BT::RuntimeError("Missing required Input: speed");
  }

  goal_.distance.data = distance_;
  goal_.speed.data = speed_;
}

BT::NodeStatus MoveForwardAction::on_success() {
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus MoveForwardAction::on_aborted() {
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus MoveForwardAction::on_cancelled() {
  // Set empty error code, action was cancelled
  return BT::NodeStatus::SUCCESS;
}
}  // namespace roast_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder = [](const std::string &name,
                               const BT::NodeConfiguration &config) {
    return std::make_unique<roast_behavior_tree::MoveForwardAction>(
        name, "move_forward", config);
  };
  factory.registerBuilder<roast_behavior_tree::MoveForwardAction>("MoveForward",
                                                                  builder);
}
