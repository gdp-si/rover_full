// This debug node is not meant to be added to the launch file
#include "roast_interfaces/srv/led_mode_selector.hpp"

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "roast_interfaces/msg/led_mode.hpp"

class LedModeSelector : public rclcpp::Node {
 public:
  LedModeSelector() : Node("led_mode_selector_service") {
    service_ = this->create_service<roast_interfaces::srv::LedModeSelector>(
        "led_mode_selector_service",
        std::bind(&LedModeSelector::led_mode_selector_service, this,
                  std::placeholders::_1, std::placeholders::_2));

    publisher_ =
        this->create_publisher<roast_interfaces::msg::LedMode>("led_mode", 10);
    RCLCPP_INFO(this->get_logger(), "Led Mode Selector Service Started");
  }

 private:
  void led_mode_selector_service(
      const std::shared_ptr<roast_interfaces::srv::LedModeSelector::Request>
          request,
      std::shared_ptr<roast_interfaces::srv::LedModeSelector::Response>
          response) {
    if (request->led_mode.led_mode) {
      RCLCPP_INFO(this->get_logger(), "Getting new LED mode");
      publisher_->publish(request->led_mode);
    }

    response->success = true;
    return;
  }

  rclcpp::Publisher<roast_interfaces::msg::LedMode>::SharedPtr publisher_;
  rclcpp::Service<roast_interfaces::srv::LedModeSelector>::SharedPtr service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LedModeSelector>());
  rclcpp::shutdown();
  return 0;
}
