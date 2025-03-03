#ifndef ROAST_LOGGING__LOGGER_CONFIG_COMPONENT_HPP_
#define ROAST_LOGGING__LOGGER_CONFIG_COMPONENT_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "roast_interfaces/srv/config_logger.hpp"
#include "roast_logging/visibility_control.hpp"

namespace roast_logging {

class LoggerConfig : public rclcpp::Node {
 public:
  ROAST_LOGGING_PUBLIC
  explicit LoggerConfig(rclcpp::NodeOptions options);

  ROAST_LOGGING_PUBLIC
  void handle_logger_config_request(
      const std::shared_ptr<roast_interfaces::srv::ConfigLogger::Request>
          request,
      std::shared_ptr<roast_interfaces::srv::ConfigLogger::Response> response);

 private:
  rclcpp::Service<roast_interfaces::srv::ConfigLogger>::SharedPtr srv_;
};

}  // namespace roast_logging

#endif  // ROAST_LOGGING__LOGGER_CONFIG_COMPONENT_HPP_
