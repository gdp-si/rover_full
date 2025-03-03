#include "roast_behavior_tree/actions/patrol.hpp"

#include <memory>
#include <string>

namespace roast_behavior_tree {
PatrolAction::PatrolAction(const std::string &xml_tag_name,
                           const std::string &action_name,
                           const BT::NodeConfiguration &config)
    : BtActionNode<Action>(xml_tag_name, action_name, config) {}

void PatrolAction::on_tick() {
  std::string patrol_type =
      getInput<std::string>("patrol_type").value_or("sequential");
  std::string patrol_mode =
      getInput<std::string>("patrol_mode").value_or("without_pause");

  // Check if the patrol type is between sequential or random
  if (std::strcmp(patrol_type.c_str(), "sequential") != 0 &&
      std::strcmp(patrol_type.c_str(), "random") != 0) {
    throw BT::RuntimeError(
        "Patrol Type should be either 'sequential' or 'random' in lower cases. "
        "Received: " +
        patrol_type + "");
  }

  if (std::strcmp(patrol_mode.c_str(), "with_pause") != 0 &&
      std::strcmp(patrol_mode.c_str(), "without_pause") != 0) {
    throw BT::RuntimeError(
        "Patrol Mode should be either 'with_pause' or 'without_pause' in lower "
        "cases. Received: " +
        patrol_mode + "");
  }

  goal_.start_patrol = true;
  goal_.patrol_type = patrol_type == "sequential"
                          ? Action::Goal::PATROL_SEQUENTIAL
                          : Action::Goal::PATROL_RANDOM;
  goal_.patrol_mode = patrol_mode == "with_pause"
                          ? Action::Goal::PATROL_WITH_PAUSE
                          : Action::Goal::PATROL_WITHOUT_PAUSE;
}

BT::NodeStatus PatrolAction::on_success() { return BT::NodeStatus::SUCCESS; }

BT::NodeStatus PatrolAction::on_aborted() { return BT::NodeStatus::FAILURE; }

BT::NodeStatus PatrolAction::on_cancelled() {
  // Set empty error code, action was cancelled
  return BT::NodeStatus::SUCCESS;
}
}  // namespace roast_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder = [](const std::string &name,
                               const BT::NodeConfiguration &config) {
    return std::make_unique<roast_behavior_tree::PatrolAction>(
        name, "patrol_action", config);
  };
  factory.registerBuilder<roast_behavior_tree::PatrolAction>("PatrolAction",
                                                             builder);
}
