#include <memory>
#include <roast_behavior_tree/actions/threat_mitigation.hpp>
#include <string>

namespace roast_behavior_tree {
ThreatMitigationService::ThreatMitigationService(
    const std::string &service_node_name, const BT::NodeConfiguration &conf)
    : BtServiceNode<roast_interfaces::srv::SprayModule>(
          service_node_name, "mitigate_threat", conf) {}

void ThreatMitigationService::on_tick() {
  request_->start_spray = true;
  getInput("spray_duration", request_->spray_duration);
  request_->spray_angle = 0.79;  // 45 degrees
}
}  // namespace roast_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<roast_behavior_tree::ThreatMitigationService>(
      "ThreatMitigationService");
}
