#include <memory>
#include <roast_behavior_tree/actions/activate_led.hpp>
#include <string>

namespace roast_behavior_tree {
ActivateLedService::ActivateLedService(const std::string &service_node_name,
                                       const BT::NodeConfiguration &conf)
    : BtServiceNode<roast_interfaces::srv::LedModeSelector>(
          service_node_name, "led_mode_selector_service", conf) {}

void ActivateLedService::on_tick() {
  std::string led_mode;
  getInput("led_mode", led_mode);

  if (led_mode == "default") {
    request_->led_mode.led_mode = request_->led_mode.DEFAULT_MODE;
  } else if (led_mode == "booted_up") {
    request_->led_mode.led_mode = request_->led_mode.ROBOT_BOOTED_UP;
  } else if (led_mode == "navigation_started") {
    request_->led_mode.led_mode = request_->led_mode.NAVIGATION_SETUP;
  } else if (led_mode == "restart") {
    request_->led_mode.led_mode = request_->led_mode.RESTART_ROBOT;
  } else if (led_mode == "threat") {
    request_->led_mode.led_mode = request_->led_mode.THREAT_DETECTED;
  } else if (led_mode == "battery_low") {
    request_->led_mode.led_mode = request_->led_mode.BATTERY_LOW;
  } else if (led_mode == "battery_critical") {
    request_->led_mode.led_mode = request_->led_mode.BATTERY_CRITICAL;
  } else if (led_mode == "navigation_stop") {
    // TODO: Better Description
    request_->led_mode.led_mode = request_->led_mode.BATTERY_LOW;
  } else {
    BT::RuntimeError("Unknown option %s", led_mode.c_str());
  }
}
}  // namespace roast_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<roast_behavior_tree::ActivateLedService>(
      "ActivateLedService");
}
