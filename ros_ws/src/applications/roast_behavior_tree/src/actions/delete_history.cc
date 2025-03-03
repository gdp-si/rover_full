#include <memory>
#include <roast_behavior_tree/actions/delete_history.hpp>
#include <string>

namespace roast_behavior_tree {
DeleteHistoryService::DeleteHistoryService(const std::string &service_node_name,
                                           const BT::NodeConfiguration &conf)
    : BtServiceNode<roast_interfaces::srv::DeleteLogs>(service_node_name,
                                                       "delete_logs", conf) {}

void DeleteHistoryService::on_tick() {
  uint8_t logs_expiry_days, type;
  std::string logs_path;
  getInput("expiry_days", logs_expiry_days);
  getInput("folder_path", logs_path);
  getInput("type", type);

  request_->expiry_days = logs_expiry_days;
  request_->folder_path = logs_path;
  request_->type = type;
}
}  // namespace roast_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<roast_behavior_tree::DeleteHistoryService>(
      "DeleteHistoryService");
}
