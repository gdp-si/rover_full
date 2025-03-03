// This debug node is not meant to be added to the launch file
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "roast_interfaces/srv/led_mode_selector.hpp"

void spray_module_service(
    const std::shared_ptr<roast_interfaces ::srv::LedModeSelector::Request>
        request,
    std::shared_ptr<roast_interfaces::srv::LedModeSelector::Response>
        response) {
  if (request->led_mode.led_mode == request->led_mode.DEFAULT_MODE) {
    RCLCPP_INFO(rclcpp::get_logger("activate_led_service_debug"),
                "Emulating led mode. DEFAULT MODE");
    response->success = true;
  } else if (request->led_mode.led_mode == request->led_mode.ROBOT_BOOTED_UP) {
    RCLCPP_INFO(rclcpp::get_logger("activate_led_service_debug"),
                "Emulating led mode. ROBOT BOOTED UP");
    response->success = false;
  } else if (request->led_mode.led_mode == request->led_mode.NAVIGATION_SETUP) {
    RCLCPP_INFO(rclcpp::get_logger("activate_led_service_debug"),
                "Emulating led mode. Navigation Setup");
    response->success = false;
  } else if (request->led_mode.led_mode == request->led_mode.RESTART_ROBOT) {
    RCLCPP_INFO(rclcpp::get_logger("activate_led_service_debug"),
                "Emulating led mode. Restart Robot");
    response->success = false;
  } else if (request->led_mode.led_mode == request->led_mode.THREAT_DETECTED) {
    RCLCPP_INFO(rclcpp::get_logger("activate_led_service_debug"),
                "Emulating led mode. Threat Detected");
    response->success = false;
  } else if (request->led_mode.led_mode == request->led_mode.BATTERY_LOW) {
    RCLCPP_INFO(rclcpp::get_logger("activate_led_service_debug"),
                "Emulating led mode. Battery low");
    response->success = false;
  }
}

int main(int argc, char **argv) {
  RCLCPP_INFO(rclcpp::get_logger("activate_led_service_debug"),
              "Activate Led Service started");
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("activate_led_service_debug_service");

  rclcpp::Service<roast_interfaces::srv::LedModeSelector>::SharedPtr service =
      node->create_service<roast_interfaces::srv::LedModeSelector>(
          "led_mode_selector_service", &spray_module_service);

  rclcpp::spin(node);
  rclcpp::shutdown();
}
