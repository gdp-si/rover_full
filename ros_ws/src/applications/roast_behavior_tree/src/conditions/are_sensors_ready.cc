#include "roast_behavior_tree/conditions/are_sensors_ready.hpp"

#include <chrono>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace roast_behavior_tree {
AreSensorsReady::AreSensorsReady(const std::string &condition_name,
                                 const BT::NodeConfiguration &conf)
    : BT::ConditionNode(condition_name, conf),
      rclcpp::Node("are_sensors_ready_condition_node")

{
  getInput("master_lifecycle_node", master_lifecycle_node_);

  // Get the parameter 'node_names' from master_lifecycle_node
  status_nodes_ = this->getNodeNames(master_lifecycle_node_);

  RCLCPP_INFO(this->get_logger(), "Acquired %ld nodes from %s",
              status_nodes_.size(), master_lifecycle_node_.c_str());
  RCLCPP_INFO(this->get_logger(), "Node %s initialized",
              condition_name.c_str());
}

bool AreSensorsReady::checkSensorStatus(std::string service_name,
                                        std::chrono::seconds timeout = 3s) {
  auto client =
      this->create_client<lifecycle_msgs::srv::GetState>(service_name);
  if (!client->wait_for_service(timeout)) {
    RCLCPP_ERROR(this->get_logger(), "Service %s not available after waiting",
                 service_name.c_str());
    return false;
  }

  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  auto future = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                         future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to call service %s",
                 service_name.c_str());
    return false;
  }

  auto response = future.get();
  if (response->current_state.id !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_ERROR(this->get_logger(), "Service %s not active",
                 service_name.c_str());
    return false;
  }

  return true;
}

std::vector<std::string> AreSensorsReady::getNodeNames(
    std::string master_node) {
  auto client = this->create_client<rcl_interfaces::srv::GetParameters>(
      master_node + "/get_parameters");
  if (!client->wait_for_service(3s)) {
    RCLCPP_ERROR(this->get_logger(), "Service %s not available after waiting",
                 (master_node + "/get_parameters").c_str());
    return std::vector<std::string>();
  }

  auto request =
      std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  request->names.push_back("node_names");
  auto future = client->async_send_request(request);

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                         future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to call service %s",
                 (master_node + "/get_parameters").c_str());
    return std::vector<std::string>();
  }

  auto response = future.get();
  if (response->values.size() == 0) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to get parameter 'node_names' from %s",
                 master_node.c_str());
    return std::vector<std::string>();
  }

  return response->values[0].string_array_value;
}

bool AreSensorsReady::checkNodeStatus(std::string node_name) {
  auto client = this->create_client<lifecycle_msgs::srv::GetState>(
      node_name + "/get_state");
  if (!client->wait_for_service(3s)) {
    RCLCPP_ERROR(this->get_logger(), "Service %s not available after waiting",
                 (node_name + "/get_state").c_str());
    return false;
  }

  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  auto future = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                         future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to call service %s",
                 (node_name + "/get_state").c_str());
    return false;
  }

  auto response = future.get();
  if (response->current_state.id !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_ERROR(this->get_logger(), "Service %s not active",
                 (node_name + "/get_state").c_str());
    return false;
  }

  return true;
}

BT::NodeStatus AreSensorsReady::tick() {
  // Check if there are nodes to check
  if (status_nodes_.size() == 0) {
    RCLCPP_INFO(this->get_logger(), "No more nodes to check");
    return BT::NodeStatus::SUCCESS;
  }

  // Check if nodes are active
  std::string node = status_nodes_[0];
  status_nodes_.erase(status_nodes_.begin());
  if (!this->checkNodeStatus(node)) {
    RCLCPP_INFO(this->get_logger(), "Node %s not active", node.c_str());
    return BT::NodeStatus::FAILURE;
  } else {
    RCLCPP_INFO(this->get_logger(), "Node %s active", node.c_str());
    return BT::NodeStatus::RUNNING;
  }
}

}  // namespace roast_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<roast_behavior_tree::AreSensorsReady>(
      "AreSensorsReady");
}
