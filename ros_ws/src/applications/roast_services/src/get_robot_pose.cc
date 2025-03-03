#include "roast_interfaces/srv/get_robot_pose.hpp"

#include <memory>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class GetRobotPose : public rclcpp::Node {
 public:
  GetRobotPose() : Node("get_robot_pose_service") {
    // Get odometry topic as parameter
    odometry_topic_ =
        this->declare_parameter("odometry_topic", "odometry/filtered");

    // Create service
    service_ = this->create_service<roast_interfaces::srv::GetRobotPose>(
        "get_robot_pose",
        std::bind(&GetRobotPose::get_robot_pose, this, std::placeholders::_1,
                  std::placeholders::_2));

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic_, 1,
        std::bind(&GetRobotPose::odometry_callback, this,
                  std::placeholders::_1));
    // Wait for the odometry topic to be available
    while (!this->count_subscribers(odometry_topic_)) {
      RCLCPP_INFO(this->get_logger(),
                  "Waiting for odometry `%s` to be available",
                  odometry_topic_.c_str());
      rclcpp::sleep_for(std::chrono::seconds(1));
    }

    RCLCPP_INFO(this->get_logger(), "Get Robot Pose Service Started");
  }

 private:
  void get_odometry_data() {
    // Get paramter if it is updated
    odometry_topic_ = this->get_parameter_or("odometry_topic", odometry_topic_);

    // Subscribe to odometry topic
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic_, 1,
        std::bind(&GetRobotPose::odometry_callback, this,
                  std::placeholders::_1));

    // Wait for odometry data
    while (!new_odometry_data_) {
      RCLCPP_INFO(this->get_logger(), "Waiting for odometry data: `%s`",
                  odometry_topic_.c_str());
      rclcpp::sleep_for(std::chrono::seconds(1));
    }

    // Unsubscribe from odometry topic
    subscription_.reset();
    new_odometry_data_ = false;
  }

  void get_robot_pose(
      const std::shared_ptr<roast_interfaces::srv::GetRobotPose::Request>
          request,
      std::shared_ptr<roast_interfaces::srv::GetRobotPose::Response> response) {
    if (request->get_current_pose == true) {
      // Get paramter if it is updated
      odometry_topic_ =
          this->get_parameter_or("odometry_topic", odometry_topic_);

      // Push request to thread
      std::thread t(&GetRobotPose::get_odometry_data, this);
      t.detach();

      // Push response
      RCLCPP_INFO(this->get_logger(), "Getting robot Pose");
      response->robot_pose.header = odometry_data_->header;
      response->robot_pose.pose = odometry_data_->pose.pose;
      RCLCPP_DEBUG(this->get_logger(), "Robot Pose: %f, %f, %f",
                   response->robot_pose.pose.position.x,
                   response->robot_pose.pose.position.y,
                   response->robot_pose.pose.position.z);

      // Stop thread
      t.~thread();
    }
  }

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "Odometry received");
    odometry_data_ = msg;
    new_odometry_data_ = true;
  }
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  nav_msgs::msg::Odometry::SharedPtr odometry_data_;
  rclcpp::Service<roast_interfaces::srv::GetRobotPose>::SharedPtr service_;
  std::string odometry_topic_;
  bool new_odometry_data_ = false;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GetRobotPose>());
  rclcpp::shutdown();
  return 0;
}
