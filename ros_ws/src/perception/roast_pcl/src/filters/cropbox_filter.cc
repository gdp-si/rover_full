#include <Eigen/Eigen>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <roast_pcl/filters/cropbox_filter.hpp>

namespace roast_pcl {
CropboxFilter::CropboxFilter(const rclcpp::NodeOptions &options)
    : FilterCommon("cropbox_filter", options) {
  // Create a parameter for the filter
  this->declare_parameter<double>("min_x", -1.0);
  this->declare_parameter<double>("min_y", -1.0);
  this->declare_parameter<double>("min_z", -1.0);
  this->declare_parameter<double>("max_x", 1.0);
  this->declare_parameter<double>("max_y", 1.0);
  this->declare_parameter<double>("max_z", 1.0);
  this->declare_parameter<bool>("invert", true);

  timer_ =
      this->create_wall_timer(std::chrono::milliseconds(100),
                              std::bind(&CropboxFilter::timer_callback, this));
  RCLCPP_INFO(this->get_logger(), "Cropbox filter node has been initialized.");
}

CropboxFilter::~CropboxFilter() {}

void CropboxFilter::timer_callback() {
  // Wait for the input cloud to be ready
  if (!input_filter_) {
    RCLCPP_WARN_ONCE(
        this->get_logger(),
        "No input cloud available yet. Waiting for the first input cloud...");
    return;
  }
  RCLCPP_INFO_ONCE(this->get_logger(), "Cropbox filter node has been started.");

  // Add the input cloud the filter
  cropbox_.setInputCloud(input_filter_);

  Eigen::Vector4f min_point, max_point;
  min_point[0] = this->get_parameter("min_x").as_double();
  min_point[1] = this->get_parameter("min_y").as_double();
  min_point[2] = this->get_parameter("min_z").as_double();
  max_point[0] = this->get_parameter("max_x").as_double();
  max_point[1] = this->get_parameter("max_y").as_double();
  max_point[2] = this->get_parameter("max_z").as_double();
  bool invert = this->get_parameter("invert").as_bool();
  cropbox_.setMin(min_point);
  cropbox_.setMax(max_point);
  cropbox_.setNegative(invert);

  // Filter the cloud
  cropbox_.filter(*output_filter_);

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

RCLCPP_COMPONENTS_REGISTER_NODE(roast_pcl::CropboxFilter)
