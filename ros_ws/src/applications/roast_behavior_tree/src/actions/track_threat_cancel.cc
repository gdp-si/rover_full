#include "roast_behavior_tree/actions/track_threat_cancel.hpp"

#include <memory>
#include <string>

namespace roast_behavior_tree {

TrackThreatCancel::TrackThreatCancel(const std::string &xml_tag_name,
                                     const std::string &action_name,
                                     const BT::NodeConfiguration &conf)
    : BtCancelActionNode<roast_interfaces::action::TrackTarget>(
          xml_tag_name, action_name, conf) {}

}  // namespace roast_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder = [](const std::string &name,
                               const BT::NodeConfiguration &config) {
    return std::make_unique<roast_behavior_tree::TrackThreatCancel>(
        name, "track_threat", config);
  };

  factory.registerBuilder<roast_behavior_tree::TrackThreatCancel>(
      "CancelTrackThreat", builder);
}
