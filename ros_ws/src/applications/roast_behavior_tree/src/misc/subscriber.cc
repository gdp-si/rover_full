#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/blackboard.h"
#include "rclcpp/rclcpp.hpp"
#include "roast_interfaces/msg/threat_info.hpp"

namespace roast_subscriber {
class RoastSubscription : public rclcpp::Node {
 public:
  RoastSubscription() : Node("roast_subscriber") {
    using namespace std::placeholders;
    using namespace std::chrono_literals;
    topic_name_ = "threat_info";
    blackboard_key_ = "threat_spin_angle";
    blackboard_ = BT::Blackboard::create();
    subscription_ =
        this->create_subscription<roast_interfaces::msg::ThreatInfo>(
            topic_name_, 10,
            std::bind(&RoastSubscription::threatInfoCallback, this,
                      std::placeholders::_1));
  }

 private:
  void threatInfoCallback(
      const roast_interfaces::msg::ThreatInfo::SharedPtr msg) {
    if (msg->near_safety_perimeter) {
      RCLCPP_DEBUG(this->get_logger(), "Setting threat yaw %f",
                   msg->threat_yaw);
      blackboard_->set("threat_spin_angle", msg->threat_yaw);
    }
  }
  rclcpp::Subscription<roast_interfaces::msg::ThreatInfo>::SharedPtr
      subscription_;
  std::string topic_name_;
  std::string blackboard_key_;
  double threat_yaw;
  BT::Blackboard::Ptr blackboard_;
};
}  // namespace roast_subscriber

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<roast_subscriber::RoastSubscription>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
