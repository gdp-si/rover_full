#ifndef ROAST_LOGGING__LOGGER_USAGE_COMPONENT_HPP_
#define ROAST_LOGGING__LOGGER_USAGE_COMPONENT_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "roast_logging/visibility_control.hpp"
#include "std_msgs/msg/string.hpp"

namespace roast_logging {

class LoggerUsage : public rclcpp::Node {
 public:
  ROAST_LOGGING_PUBLIC
  explicit LoggerUsage(rclcpp::NodeOptions options);

 protected:
  void on_timer();

 private:
  size_t count_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr one_shot_timer_, timer_;
  std::function<bool()> debug_function_to_evaluate_;
};

bool is_divisor_of_twelve(size_t val, rclcpp::Logger logger);
}  // namespace roast_logging

#endif  // ROAST_LOGGING__LOGGER_USAGE_COMPONENT_HPP_
