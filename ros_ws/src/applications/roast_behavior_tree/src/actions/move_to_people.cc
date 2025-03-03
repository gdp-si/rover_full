#include "roast_behavior_tree/actions/move_to_people.hpp"

#include <memory>

namespace roast_behavior_tree {
MoveToPeopleGroup::MoveToPeopleGroup(const std::string &xml_tag_name,
                                     const std::string &action_name,
                                     const BT::NodeConfiguration &config)
    : BtActionNode<Action>(xml_tag_name, action_name, config) {
  turn_to_group_ = getInput<bool>("turn_to_group").value_or(false);
  topic_timeout_ = getInput<double>("topic_timeout").value_or(3.0);

  callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
  executor_.add_callback_group(callback_group_,
                               node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  group_info_sub_ =
      node_->create_subscription<roast_interfaces::msg::PeopleGroup>(
          "people_info", rclcpp::SystemDefaultsQoS(),
          std::bind(&MoveToPeopleGroup::groupInfoCallback, this,
                    std::placeholders::_1),
          sub_option);

  // Add the executor spin function to a separate thread
  executor_thread_ = std::thread([this]() { executor_.spin(); });
}

MoveToPeopleGroup::~MoveToPeopleGroup() {
  // Close the subscription and cancel the executor thread
  executor_.cancel();
  executor_thread_.join();
}

void MoveToPeopleGroup::groupInfoCallback(const GroupInfo::SharedPtr msg) {
  group_info_ = *msg;
}

void MoveToPeopleGroup::on_tick() {
  goal_.move_to_group = true;
  goal_.turn_to_group = turn_to_group_;
  goal_.topic_timeout = topic_timeout_;
}

BT::NodeStatus MoveToPeopleGroup::on_success() {
  if (group_info_.group_detected)
    setOutput("group_orientation", group_info_.group_orientation);
  else
    setOutput("group_orientation", 0.0);

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus MoveToPeopleGroup::on_aborted() {
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus MoveToPeopleGroup::on_cancelled() {
  // Set empty error code, action was cancelled
  return BT::NodeStatus::SUCCESS;
}
}  // namespace roast_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder = [](const std::string &name,
                               const BT::NodeConfiguration &config) {
    return std::make_unique<roast_behavior_tree::MoveToPeopleGroup>(
        name, "move_to_people", config);
  };
  factory.registerBuilder<roast_behavior_tree::MoveToPeopleGroup>(
      "MoveToPeopleGroup", builder);
}
