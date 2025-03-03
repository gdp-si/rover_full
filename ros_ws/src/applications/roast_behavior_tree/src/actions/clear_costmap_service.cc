#include "roast_behavior_tree/actions/clear_costmap_service.hpp"

#include <memory>
#include <string>

namespace roast_behavior_tree {

ClearEntireCostmapService::ClearEntireCostmapService(
    const std::string &service_node_name, const std::string &service_name,
    const BT::NodeConfiguration &conf)
    : BtServiceNode<nav2_msgs::srv::ClearEntireCostmap>(service_node_name,
                                                        service_name, conf) {}

void ClearEntireCostmapService::on_tick() { increment_recovery_count(); }

ClearCostmapExceptRegionService::ClearCostmapExceptRegionService(
    const std::string &service_node_name, const std::string &service_name,
    const BT::NodeConfiguration &conf)
    : BtServiceNode<nav2_msgs::srv::ClearCostmapExceptRegion>(
          service_node_name, service_name, conf) {}

void ClearCostmapExceptRegionService::on_tick() {
  getInput("reset_distance", request_->reset_distance);
  increment_recovery_count();
}

ClearCostmapAroundRobotService::ClearCostmapAroundRobotService(
    const std::string &service_node_name, const std::string &service_name,
    const BT::NodeConfiguration &conf)
    : BtServiceNode<nav2_msgs::srv::ClearCostmapAroundRobot>(
          service_node_name, service_name, conf) {}

void ClearCostmapAroundRobotService::on_tick() {
  getInput("reset_distance", request_->reset_distance);
  increment_recovery_count();
}

}  // namespace roast_behavior_tree

#include <map>

#include "behaviortree_cpp_v3/bt_factory.h"
template <typename NodeType>
void registerBTNode(std::map<std::string, std::string> service_topics,
                    BT::BehaviorTreeFactory &factory) {
  for (auto &pair : service_topics) {
    const std::string &action_name = pair.first;
    const std::string &service_name = pair.second;
    BT::NodeBuilder builder = [&, service_name](
                                  const std::string &name,
                                  const BT::NodeConfiguration &config) {
      return std::make_unique<NodeType>(name, service_name, config);
    };
    factory.registerBuilder<NodeType>(action_name, builder);
  }
  return;
};
BT_REGISTER_NODES(factory) {
  // Dictionary for the Action name and topic name
  std::map<std::string, std::string> service_topics = {
      {"ClearEntireLocalCostmap", "local_costmap/clear_entirely_local_costmap"},
      {"ClearEntireGlobalCostmap",
       "global_costmap/clear_entirely_global_costmap"},
  };
  registerBTNode<roast_behavior_tree::ClearEntireCostmapService>(service_topics,
                                                                 factory);

  service_topics = {
      {"ClearLocalCostmapExceptRegion",
       "local_costmap/clear_except_local_costmap"},
      {"ClearGlobalCostmapExceptRegion",
       "global_costmap/clear_except_global_costmap"},
  };
  registerBTNode<roast_behavior_tree::ClearCostmapExceptRegionService>(
      service_topics, factory);

  service_topics = {
      {"ClearLocalCostmapAroundRobot",
       "local_costmap/clear_around_local_costmap"},
      {"ClearGlobalCostmapAroundRobot",
       "global_costmap/clear_around_global_costmap"},
  };
  registerBTNode<roast_behavior_tree::ClearCostmapAroundRobotService>(
      service_topics, factory);
}
