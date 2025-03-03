// This debug node is not meant to be added to the launch file
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "roast_interfaces/msg/display_mode.hpp"
#include "roast_interfaces/srv/display.hpp"

class DisplayModeSelector : public rclcpp::Node {
 public:
  DisplayModeSelector() : Node("display_mode_selector_service") {
    service_ = this->create_service<roast_interfaces::srv::Display>(
        "display_mode_selector_service",
        std::bind(&DisplayModeSelector::display_mode_selector_service, this,
                  std::placeholders::_1, std::placeholders::_2));

    publisher_ = this->create_publisher<roast_interfaces::msg::DisplayMode>(
        "display_mode", 10);
    RCLCPP_INFO(this->get_logger(), "Display Mode Selector Service Started");
  }

 private:
  void display_mode_selector_service(
      const std::shared_ptr<roast_interfaces::srv::Display::Request> request,
      std::shared_ptr<roast_interfaces::srv::Display::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Getting new Display mode");
    publisher_->publish(request->mode);

    response->success = true;
    return;
  }

  rclcpp::Publisher<roast_interfaces::msg::DisplayMode>::SharedPtr publisher_;
  rclcpp::Service<roast_interfaces::srv::Display>::SharedPtr service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DisplayModeSelector>());
  rclcpp::shutdown();
  return 0;
}
