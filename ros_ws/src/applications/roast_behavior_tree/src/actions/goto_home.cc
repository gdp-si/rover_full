#include "roast_behavior_tree/actions/goto_home.hpp"

#include <memory>
#include <string>

#include "rclcpp/parameter.hpp"

namespace roast_behavior_tree {
GoToHomeAction::GoToHomeAction(const std::string &xml_tag_name,
                               const std::string &action_name,
                               const BT::NodeConfiguration &config)
    : BtActionNode<Action>(xml_tag_name, action_name, config) {}

void GoToHomeAction::on_tick() {
  getInput<double>("backup_distance", backup_distance_);
  getInput<double>("spin_angle", spin_angle_);

  if (!backup_distance_) {
    throw BT::RuntimeError("Missing required Input: backup_distance");
  }

  if (!spin_angle_) {
    throw BT::RuntimeError("Missing required Input: spin_angle");
  }

  // Set backup distance and spin angle as action server parameters
  // so that they can be accessed by the action server
  rclcpp::Parameter backup_distance_param("home_backup_distance",
                                          backup_distance_);
  rclcpp::Parameter spin_angle_param("home_spin_angle", spin_angle_);
}

BT::NodeStatus GoToHomeAction::on_success() { return BT::NodeStatus::SUCCESS; }

BT::NodeStatus GoToHomeAction::on_aborted() { return BT::NodeStatus::FAILURE; }

BT::NodeStatus GoToHomeAction::on_cancelled() {
  // Set empty error code, action was cancelled
  return BT::NodeStatus::SUCCESS;
}
}  // namespace roast_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder = [](const std::string &name,
                               const BT::NodeConfiguration &config) {
    return std::make_unique<roast_behavior_tree::GoToHomeAction>(
        name, "goto_home", config);
  };
  factory.registerBuilder<roast_behavior_tree::GoToHomeAction>("GoToHome",
                                                               builder);
}
