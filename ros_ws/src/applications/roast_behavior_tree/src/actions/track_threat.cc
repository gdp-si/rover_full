#include "roast_behavior_tree/actions/track_threat.hpp"

#include <memory>

namespace roast_behavior_tree {
TrackThreat::TrackThreat(const std::string &xml_tag_name,
                         const std::string &action_name,
                         const BT::NodeConfiguration &config)
    : BtActionNode<Action>(xml_tag_name, action_name, config) {
  turn_to_threat_ = getInput<bool>("turn_to_threat").value_or(false);
  topic_timeout_ = getInput<double>("topic_timeout").value_or(3.0);

  callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
  executor_.add_callback_group(callback_group_,
                               node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  threat_info_sub_ =
      node_->create_subscription<roast_interfaces::msg::ThreatInfo>(
          "threat_info", rclcpp::SystemDefaultsQoS(),
          std::bind(&TrackThreat::threatInfoCallback, this,
                    std::placeholders::_1),
          sub_option);

  // Add the executor spin function to a separate thread
  executor_thread_ = std::thread([this]() { executor_.spin(); });
}

TrackThreat::~TrackThreat() {
  // Close the subscription and cancel the executor thread
  executor_.cancel();
  executor_thread_.join();
}

void TrackThreat::threatInfoCallback(const ThreatInfo::SharedPtr msg) {
  threat_info_ = *msg;
}

void TrackThreat::on_tick() {
  goal_.start_tracking = true;
  goal_.turn_to_target = turn_to_threat_;
  goal_.topic_timeout = topic_timeout_;
}

BT::NodeStatus TrackThreat::on_success() {
  if (threat_info_.near_safety_perimeter)
    setOutput("threat_orientation", threat_info_.threat_yaw);
  else
    setOutput("threat_orientation", 0.0);

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus TrackThreat::on_aborted() { return BT::NodeStatus::FAILURE; }

BT::NodeStatus TrackThreat::on_cancelled() {
  // Set empty error code, action was cancelled
  return BT::NodeStatus::SUCCESS;
}
}  // namespace roast_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder = [](const std::string &name,
                               const BT::NodeConfiguration &config) {
    return std::make_unique<roast_behavior_tree::TrackThreat>(
        name, "track_threat", config);
  };
  factory.registerBuilder<roast_behavior_tree::TrackThreat>("TrackThreat",
                                                            builder);
}
