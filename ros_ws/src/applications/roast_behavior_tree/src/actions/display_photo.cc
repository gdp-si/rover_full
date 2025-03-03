#include <sys/stat.h>

#include <cstdlib>
#include <fstream>
#include <memory>
#include <roast_behavior_tree/actions/display_photo.hpp>
#include <string>

namespace roast_behavior_tree {

// get data from the filepath

DisplayPhotoBTNode::DisplayPhotoBTNode(const std::string &service_node_name,
                                       const BT::NodeConfiguration &conf)
    : BtServiceNode<roast_interfaces::srv::DisplayPhoto>(
          service_node_name, "display_photo", conf) {
  // Server timeout
  server_timeout_ = std::chrono::milliseconds(
      static_cast<int>(server_timeout_seconds_ * 1000));

  // Get the environment variable ROAST_HOME
  char *roast_home = std::getenv("ROAST_HOME");
  if (roast_home == NULL) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "ROAST_HOME environment variable not set");
    auto home_dir = std::getenv("HOME");
    roast_home = (char *)malloc(strlen(home_dir) + strlen("/.roast") + 1);
  }
  root_dir_ = std::string(roast_home) + "/media";
  filename_ = root_dir_ + "/latest_image.txt";
}

void DisplayPhotoBTNode::on_tick() {
  time_interval_ = getInput<double>("time_interval").value_or(5.0);

  if (time_interval_ > server_timeout_seconds_) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                "Time interval %f is greater than server timeout %f",
                time_interval_, server_timeout_seconds_);
    time_interval_ = server_timeout_seconds_;
  }

  auto file_data = getFileData(filename_);
  auto image_data = getImageData(file_data[0]);

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Displaying image %s",
               file_data[0].c_str());

  request_->image = *image_data->toImageMsg();
  request_->time_interval = time_interval_;
  request_->show_raw_image = false;
}

std::vector<std::string> DisplayPhotoBTNode::getFileData(
    const std::string &filename) {
  std::vector<std::string> file_data;

  // read file contents
  std::ifstream file(filename);
  if (file.is_open()) {
    std::string line;
    while (std::getline(file, line)) {
      file_data.push_back(line);
    }
    file.close();
  } else {
    std::cerr << "Unable to open file " << filename << " for reading"
              << std::endl;
  }

  return file_data;
}

cv_bridge::CvImagePtr DisplayPhotoBTNode::getImageData(
    const std::string &filename) {
  cv_bridge::CvImagePtr cv_ptr;

  // read image
  cv::Mat image = cv::imread(filename, cv::IMREAD_COLOR);
  if (image.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unable to open image %s",
                 filename.c_str());
  } else {
    cv_ptr = std::make_shared<cv_bridge::CvImage>();
    cv_ptr->encoding = "bgr8";
    cv_ptr->image = image;
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Image size: %d x %d",
                 image.cols, image.rows);
  }

  return cv_ptr;
}
}  // namespace roast_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<roast_behavior_tree::DisplayPhotoBTNode>(
      "DisplayPhoto");
}
