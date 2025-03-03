// This debug node is not meant to be added to the launch file
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "roast_interfaces/srv/spray_module.hpp"

void spray_module_service(
    const std::shared_ptr<roast_interfaces ::srv::SprayModule::Request> request,
    std::shared_ptr<roast_interfaces::srv::SprayModule::Response> response) {
  if (request->start_spray) {
    RCLCPP_INFO(rclcpp::get_logger("threat_mitigation"),
                "Emulating spray module. Success");
    response->success = true;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("threat_mitigation"),
                "Emulating spray module. Failure");
    response->success = false;
  }
}

int main(int argc, char **argv) {
  RCLCPP_INFO(rclcpp::get_logger("threat_mitigation"),
              "Threat Mitigation Service started");
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("threat_mitigation_service");

  rclcpp::Service<roast_interfaces::srv::SprayModule>::SharedPtr service =
      node->create_service<roast_interfaces::srv::SprayModule>(
          "threat_spray_service", &spray_module_service);

  rclcpp::spin(node);
  rclcpp::shutdown();
}
