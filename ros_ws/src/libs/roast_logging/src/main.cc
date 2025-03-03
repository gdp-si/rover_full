
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "roast_logging/logger_config_component.hpp"
#include "roast_logging/logger_usage_component.hpp"

int main(int argc, char *argv[]) {
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  // Create a node that processes logger configuration requests
  auto logger_config = std::make_shared<roast_logging::LoggerConfig>(options);
  exec.add_node(logger_config);
  // Create a node that has examples of different logger usage
  auto logger_usage = std::make_shared<roast_logging::LoggerUsage>(options);
  exec.add_node(logger_usage);

  exec.spin();

  rclcpp::shutdown();
  return 0;
}
