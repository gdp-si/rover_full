#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <roast_pcl/filters/voxel_filter.hpp>
#include <string>

namespace roast_pcl {
VoxelGridFilter::VoxelGridFilter(const rclcpp::NodeOptions &options)
    : FilterCommon("voxel_filter", options) {
  // Create a parameter for the filter leaf size
  this->declare_parameter<double>("leaf_size", 0.1);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&VoxelGridFilter::timer_callback, this));
  RCLCPP_INFO(this->get_logger(), "Voxel filter node has been initialized.");
}

VoxelGridFilter::~VoxelGridFilter() {}

void VoxelGridFilter::timer_callback() {
  // Wait for the input cloud to be ready
  if (!input_filter_) {
    RCLCPP_WARN_ONCE(
        this->get_logger(),
        "No input cloud available yet. Waiting for the first input cloud...");
    return;
  }
  RCLCPP_INFO_ONCE(this->get_logger(), "Voxel filter node has been started.");

  // Add the input cloud the filter
  voxel_.setInputCloud(input_filter_);

  // Get the filter field name parameter
  double leaf_size = this->get_parameter("leaf_size").as_double();
  voxel_.setLeafSize(leaf_size, leaf_size, leaf_size);

  // Filter the cloud
  voxel_.filter(*output_filter_);

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

RCLCPP_COMPONENTS_REGISTER_NODE(roast_pcl::VoxelGridFilter)
