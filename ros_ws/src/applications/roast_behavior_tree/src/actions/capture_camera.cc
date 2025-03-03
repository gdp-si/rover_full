#include <memory>
#include <roast_behavior_tree/actions/capture_camera.hpp>
#include <string>

namespace roast_behavior_tree {
CapturePhotoBTNode::CapturePhotoBTNode(const std::string &service_node_name,
                                       const BT::NodeConfiguration &conf)
    : BtServiceNode<roast_interfaces::srv::CaptureCamera>(
          service_node_name, "capture_camera_service", conf) {}

void CapturePhotoBTNode::on_tick() {
  getInput("image_topic", request_->image_topic);
  getInput("timeout", request_->timeout);

  request_->capture_photo = true;
}

}  // namespace roast_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<roast_behavior_tree::CapturePhotoBTNode>(
      "CaptureCamera");
}
