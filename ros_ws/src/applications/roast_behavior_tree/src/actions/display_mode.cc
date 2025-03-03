#include <memory>
#include <roast_behavior_tree/actions/display_mode.hpp>
#include <string>

namespace roast_behavior_tree {
DisplayModeService::DisplayModeService(const std::string &service_node_name,
                                       const BT::NodeConfiguration &conf)
    : BtServiceNode<roast_interfaces::srv::Display>(
          service_node_name, "display_mode_selector_service", conf) {}

void DisplayModeService::on_tick() {
  std::string display_mode;
  getInput("display_mode", display_mode);

  if (display_mode == "blink") {
    request_->mode.display_mode = request_->mode.MODE_BLINK;
  } else if (display_mode == "blink_thrice") {
    request_->mode.display_mode = request_->mode.MODE_BLINK_THRICE;
  } else if (display_mode == "patrol") {
    request_->mode.display_mode = request_->mode.MODE_PATROL;
  } else if (display_mode == "bootup") {
    request_->mode.display_mode = request_->mode.MODE_BOOTUP;
  } else if (display_mode == "threat_tracking_on") {
    request_->mode.display_mode = request_->mode.MODE_THREAT_TRACKING_ON;
  } else if (display_mode == "threat_tracking_off") {
    request_->mode.display_mode = request_->mode.MODE_THREAT_TRACKING_OFF;
  } else {
    BT::RuntimeError("Unknown option %s", display_mode.c_str());
  }
}
}  // namespace roast_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<roast_behavior_tree::DisplayModeService>(
      "DisplayMode");
}
