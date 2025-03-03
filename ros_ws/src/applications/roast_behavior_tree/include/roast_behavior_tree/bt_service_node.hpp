#ifndef ROAST_BEHAVIOR_TREE__BT_SERVICE_NODE_HPP_
#define ROAST_BEHAVIOR_TREE__BT_SERVICE_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "roast_behavior_tree/bt_conversions.hpp"

namespace roast_behavior_tree {

using namespace std::chrono_literals;  // NOLINT

/**
 * @brief Abstract class representing a service based BT node
 * @tparam ServiceT Type of service
 */
template <class ServiceT>
class BtServiceNode : public BT::ActionNodeBase {
 public:
  /**
   * @brief A roast_behavior_tree::BtServiceNode constructor
   * @param service_node_name Service name this node creates a client for
   * @param conf BT node configuration
   */
  BtServiceNode(const std::string &service_node_name,
                const std::string &service_name,
                const BT::NodeConfiguration &conf)
      : BT::ActionNodeBase(service_node_name, conf),
        service_node_name_(service_node_name),
        service_name_(service_name) {
    if (!config().blackboard->getAny("node")) {
      RCLCPP_WARN(
          rclcpp::get_logger("BTServiceNode"),
          "A node pointer was not found on the blackboard. Using a new node.");
      node_ = rclcpp::Node::make_shared(service_node_name + "_node");
    } else {
      node_ =
          config().blackboard->template get<rclcpp::Node::SharedPtr>("node");
    }
    callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(
        callback_group_, node_->get_node_base_interface());

    // Get the required items from the blackboard
    if (!config().blackboard->getAny("bt_loop_duration")) {
      RCLCPP_WARN(node_->get_logger(),
                  "A loop duration was not found on the blackboard. Using "
                  "default value.");
      bt_loop_duration_ = std::chrono::milliseconds(100);  // 10 Hz
    } else {
      bt_loop_duration_ =
          config().blackboard->template get<std::chrono::milliseconds>(
              "bt_loop_duration");
    }
    if (!config().blackboard->getAny("server_timeout")) {
      RCLCPP_WARN(node_->get_logger(),
                  "A server timeout was not found on the blackboard. Using "
                  "default value.");
      server_timeout_ = std::chrono::milliseconds(3000);  // 3 s
    } else {
      server_timeout_ =
          config().blackboard->template get<std::chrono::milliseconds>(
              "server_timeout");
    }
    getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);

    // Now that we have node_ to use, create the service client for this BT
    // service
    service_client_ = node_->create_client<ServiceT>(
        service_name_, rmw_qos_profile_services_default, callback_group_);

    // Make a request for the service without parameter
    request_ = std::make_shared<typename ServiceT::Request>();

    // Make sure the server is actually there before continuing
    RCLCPP_INFO(node_->get_logger(), "Waiting for \"%s\" service",
                service_name_.c_str());
    if (!service_client_->wait_for_service(server_timeout_)) {
      RCLCPP_ERROR(node_->get_logger(),
                   "\"%s\" service not available after waiting for %ld s",
                   service_name_.c_str(), server_timeout_.count());
      throw std::runtime_error(std::string("Service server %s not available",
                                           service_node_name.c_str()));
    }

    RCLCPP_DEBUG(node_->get_logger(), "\"%s\" BtServiceNode initialized",
                 service_node_name_.c_str());
  }

  BtServiceNode() = delete;

  virtual ~BtServiceNode() {}

  /**
   * @brief Any subclass of BtServiceNode that accepts parameters must provide a
   * providedPorts method and call providedBasicPorts in it.
   * @param addition Additional ports to add to BT port list
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedBasicPorts(BT::PortsList addition) {
    BT::PortsList basic = {
        BT::InputPort<std::chrono::milliseconds>("server_timeout")};
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts() { return providedBasicPorts({}); }

  /**
   * @brief The main override required by a BT service
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override {
    if (!request_sent_) {
      on_tick();
      future_result_ =
          service_client_->async_send_request(request_).future.share();
      sent_time_ = node_->now();
      request_sent_ = true;
    }
    return check_future();
  }

  /**
   * @brief The other (optional) override required by a BT service.
   */
  void halt() override {
    request_sent_ = false;
    setStatus(BT::NodeStatus::IDLE);
  }

  /**
   * @brief Function to perform some user-defined operation on tick
   * Fill in service request with information if necessary
   */
  virtual void on_tick() {}

  /**
   * @brief Function to perform some user-defined operation upon successful
   * completion of the service. Could put a value on the blackboard.
   * @param response can be used to get the result of the service call in the BT
   * Node.
   * @return BT::NodeStatus Returns SUCCESS by default, user may override to
   * return another value
   */
  virtual BT::NodeStatus on_completion(
      std::shared_ptr<typename ServiceT::Response> /*response*/) {
    return BT::NodeStatus::SUCCESS;
  }

  /**
   * @brief Check the future and decide the status of BT
   * @return BT::NodeStatus SUCCESS if future complete before timeout, FAILURE
   * otherwise
   */
  virtual BT::NodeStatus check_future() {
    auto elapsed =
        (node_->now() - sent_time_).to_chrono<std::chrono::milliseconds>();
    auto remaining = server_timeout_ - elapsed;

    if (remaining > std::chrono::milliseconds(0)) {
      auto timeout =
          remaining > bt_loop_duration_ ? bt_loop_duration_ : remaining;

      rclcpp::FutureReturnCode rc;
      rc = callback_group_executor_.spin_until_future_complete(future_result_,
                                                               timeout);
      if (rc == rclcpp::FutureReturnCode::SUCCESS) {
        request_sent_ = false;
        BT::NodeStatus status = on_completion(future_result_.get());
        return status;
      }

      if (rc == rclcpp::FutureReturnCode::TIMEOUT) {
        on_wait_for_result();
        elapsed =
            (node_->now() - sent_time_).to_chrono<std::chrono::milliseconds>();
        if (elapsed < server_timeout_) {
          return BT::NodeStatus::RUNNING;
        }
      }
    }

    RCLCPP_WARN(node_->get_logger(),
                "Node timed out while executing service call to %s.",
                service_name_.c_str());
    request_sent_ = false;
    return BT::NodeStatus::FAILURE;
  }

  /**
   * @brief Function to perform some user-defined operation after a timeout
   * waiting for a result that hasn't been received yet
   */
  virtual void on_wait_for_result() {}

 protected:
  /**
   * @brief Function to increment recovery count on blackboard if this node
   * wraps a recovery
   */
  void increment_recovery_count() {
    int recovery_count = 0;
    config().blackboard->template get<int>("number_recoveries",
                                           recovery_count);  // NOLINT
    recovery_count += 1;
    config().blackboard->template set<int>("number_recoveries",
                                           recovery_count);  // NOLINT
  }

  std::string service_node_name_, service_name_;
  typename std::shared_ptr<rclcpp::Client<ServiceT>> service_client_;
  std::shared_ptr<typename ServiceT::Request> request_;

  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  // The timeout value while to use in the tick loop while waiting for
  // a result from the server
  std::chrono::milliseconds server_timeout_;

  // The timeout value for BT loop execution
  std::chrono::milliseconds bt_loop_duration_;

  // To track the server response when a new request is sent
  std::shared_future<typename ServiceT::Response::SharedPtr> future_result_;
  bool request_sent_{false};
  rclcpp::Time sent_time_;
};

}  // namespace roast_behavior_tree

#endif  // ROAST_BEHAVIOR_TREE__BT_SERVICE_NODE_HPP_
