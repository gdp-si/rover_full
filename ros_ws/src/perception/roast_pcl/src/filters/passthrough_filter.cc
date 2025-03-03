#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <roast_pcl/filters/passthrough_filter.hpp>
#include <string>

namespace roast_pcl {
PassthroughFilter::PassthroughFilter(const rclcpp::NodeOptions &options)
    : FilterCommon("passthrough_filter", options) {
  // Create a parameter for the filter field name
  this->declare_parameter<std::string>("filter_field_name", "z");

  // Create a parameter for the filter limits
  this->declare_parameter<double>("min_limit", 0.0);
  this->declare_parameter<double>("max_limit", 1.0);
  this->declare_parameter<bool>("invert", false);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PassthroughFilter::timer_callback, this));
  RCLCPP_INFO(this->get_logger(),
              "Passthrough filter node has been initialized.");
}

PassthroughFilter::~PassthroughFilter() {}

void PassthroughFilter::timer_callback() {
  // Wait for the input cloud to be ready
  if (!input_filter_) {
    RCLCPP_WARN_ONCE(
        this->get_logger(),
        "No input cloud available yet. Waiting for the first input cloud...");
    return;
  }
  RCLCPP_INFO_ONCE(this->get_logger(),
                   "Passthrough filter node has been started.");

  // Add the input cloud the filter
  pass_.setInputCloud(input_filter_);

  // Get the filter field name parameter
  std::string filter_field_name;
  this->get_parameter("filter_field_name", filter_field_name);
  pass_.setFilterFieldName(filter_field_name);

  // Get the filter limits parameter
  double min_limit, max_limit;
  this->get_parameter("min_limit", min_limit);
  this->get_parameter("max_limit", max_limit);
  pass_.setFilterLimits(min_limit, max_limit);

  // Get the invert parameter
  bool invert = this->get_parameter("invert").as_bool();
  pass_.setNegative(invert);

  // Filter the cloud
  pass_.filter(*output_filter_);

  // Convert the pcl::PointCloud<pcl::PointXYZ> message to a
  // sensor_msgs::msg::PointCloud2 message
  sensor_msgs::msg::PointCloud2::SharedPtr output(
      new sensor_msgs::msg::PointCloud2);
  pcl::PCLPointCloud2 *output_pcl = new pcl::PCLPointCloud2;
  pcl::toPCLPointCloud2(*output_filter_, *output_pcl);
  pcl::toROSMsg(*output_filter_, *output);

  // Publish the output message
  pub_->publish(*output);

  delete output_pcl;

  return;
}

}  // namespace roast_pcl

RCLCPP_COMPONENTS_REGISTER_NODE(roast_pcl::PassthroughFilter)
