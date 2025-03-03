#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "roast_interfaces/srv/alert_mode.hpp"
#include "std_msgs/msg/string.hpp"

class AlertMode : public rclcpp::Node {
 public:
  AlertMode() : Node("set_alert_mode") {
    service_ = this->create_service<roast_interfaces::srv::AlertMode>(
        "set_alert_mode",
        std::bind(&AlertMode::set_alert_mode, this, std::placeholders::_1,
                  std::placeholders::_2));

    sms_alert_publisher_ =
        this->create_publisher<std_msgs::msg::String>("alert_system/sms", 10);
    sound_alert_publisher_ =
        this->create_publisher<std_msgs::msg::String>("speak", 10);
    RCLCPP_INFO(this->get_logger(), "Set Alert Mode Service Started");
  }

 private:
  void set_alert_mode(
      const std::shared_ptr<roast_interfaces::srv::AlertMode::Request> request,
      std::shared_ptr<roast_interfaces::srv::AlertMode::Response> response) {
    if (request->sms == true) {
      auto message = std_msgs::msg::String();
      message.data = request->message;
      response->status = true;
      RCLCPP_INFO(this->get_logger(), "Sending SMS %s", message.data.c_str());
      sms_alert_publisher_->publish(message);
    }
    if (request->speaker == true) {
      auto message = std_msgs::msg::String();
      message.data = request->message;
      response->status = true;
      RCLCPP_INFO(this->get_logger(), "Speaker Announcement %s",
                  message.data.c_str());
      sound_alert_publisher_->publish(message);
    }
  }

  rclcpp::Service<roast_interfaces::srv::AlertMode>::SharedPtr service_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sms_alert_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sound_alert_publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AlertMode>());
  rclcpp::shutdown();
  return 0;
}
