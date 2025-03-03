#include <cv_bridge/cv_bridge.h>
#include <sys/stat.h>

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "roast_interfaces/srv/capture_camera.hpp"
#include "sensor_msgs/msg/image.hpp"

/*
This function looks for the environment variable ROAST_HOME, creates a directory
"media" if it doesn't exist, and saves the image to that directory.
*/
void save_image(const cv::Mat &image) {
  // Create media directory if it doesn't exist
  char *roast_home = std::getenv("ROAST_HOME");
  if (roast_home == NULL) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "ROAST_HOME environment variable not set");
    auto home_dir = std::getenv("HOME");
    roast_home = (char *)malloc(strlen(home_dir) + strlen("/.roast") + 1);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Creating %s/.roast directory",
                home_dir);
    strcpy(roast_home, home_dir + strlen("/.roast"));
  }

  std::string media_dir = std::string(roast_home) + "/media";

  // FIXME: It doesn't seem like this is working
  // Check if directory exists
  struct stat sb;
  if (!(stat(media_dir.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode))) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Creating media directory %s",
                media_dir.c_str());
  }

  // Create directory
  if (mkdir(media_dir.c_str(), 0777) == -1) {
    if (errno != EEXIST) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Error creating media directory");
    }
  }

  // Filename with timestamp
  auto now = std::chrono::system_clock::now();
  auto now_c = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d-%H-%M-%S");
  std::string filename = media_dir + "/image-" + ss.str() + ".jpg";

  // Write the file name to a separate file for easy access
  std::string filename_file = media_dir + "/latest_image.txt";
  std::ofstream file(filename_file);
  if (file.is_open()) {
    file << filename;
    file.close();
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unable to open file %s",
                 filename_file.c_str());
  }

  // Save image
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Saving image to %s",
              filename.c_str());
  cv::imwrite(filename, image);
}

class CaptureCameraService : public rclcpp::Node {
 public:
  CaptureCameraService() : Node("capture_camera_service") {
    // Create service
    service_ = this->create_service<roast_interfaces::srv::CaptureCamera>(
        "capture_camera_service",
        std::bind(&CaptureCameraService::capture_camera, this,
                  std::placeholders::_1, std::placeholders::_2));

    callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // Create executor
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_callback_group(callback_group_,
                                  this->get_node_base_interface());
    executor_->spin_some();

    RCLCPP_INFO(this->get_logger(), "Capture Camera Service Started");
  }

 private:
  void capture_camera(
      const std::shared_ptr<roast_interfaces::srv::CaptureCamera::Request>
          request,
      std::shared_ptr<roast_interfaces::srv::CaptureCamera::Response>
          response) {
    if (request->capture_photo == true) {
      auto timeout = request->timeout;
      auto start_time = std::chrono::steady_clock::now();

      // Subscribe to image topic
      image_topic_ = request->image_topic;

      auto sub_opt = rclcpp::SubscriptionOptions();
      sub_opt.callback_group = callback_group_;

      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
          image_topic_, 1,
          std::bind(&CaptureCameraService::image_callback, this,
                    std::placeholders::_1),
          sub_opt);

      // Send to thread
      executor_thread_ = std::thread([this]() { executor_->spin(); });

      // Wait for image data
      while (!new_image_data_) {
        RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for image data %s",
                         image_topic_.c_str());
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(
                                current_time - start_time)
                                .count();
        if (elapsed_time > timeout) {
          RCLCPP_WARN(this->get_logger(), "Timeout fetching image data");
          break;
        }
      }

      // Push response
      if (new_image_data_) {
        RCLCPP_INFO(this->get_logger(), "Getting Image");

        // Save image
        save_image(cv_ptr_->image);

        response->image = *cv_ptr_->toImageMsg();
        response->success = true;
      } else {
        response->success = false;
      }

      // Unsubscribe from image topic
      subscription_.reset();
      new_image_data_ = false;

      // Cancel executor thread
      executor_->cancel();
      executor_thread_.join();
    } else {
      RCLCPP_WARN(this->get_logger(), "No image capture requested");
      response->success = false;
    }
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Image received");
    cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    new_image_data_ = true;
  }
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Service<roast_interfaces::srv::CaptureCamera>::SharedPtr service_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Executor::SharedPtr executor_;
  cv_bridge::CvImagePtr cv_ptr_;

  std::string image_topic_;
  bool new_image_data_ = false;
  std::thread executor_thread_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CaptureCameraService>());
  rclcpp::shutdown();
  return 0;
}
