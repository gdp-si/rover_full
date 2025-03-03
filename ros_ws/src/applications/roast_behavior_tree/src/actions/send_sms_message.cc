#include "roast_behavior_tree/actions/send_sms_message.hpp"

#include <memory>
#include <string>

namespace roast_behavior_tree {
SendSmsAction::SendSmsAction(const std::string &xml_tag_name,
                             const std::string &action_name,
                             const BT::NodeConfiguration &config)
    : BtActionNode<roast_interfaces::action::SendSmsMessage>(
          xml_tag_name, action_name, config) {}

void SendSmsAction::on_tick() {
  auto message = getInput<std::string>("message");
  auto level = getInput<std::string>("level");
  auto frequency = getInput<int>("frequency");

  if (!message) {
    throw BT::RuntimeError("missing required input [message]: ",
                           message.error());
  }

  if (!level) {
    throw BT::RuntimeError("missing required input [level]: ", level.error());
  }

  goal_.message = message.value();

  if (level.value() == "INFO") {
    goal_.message_level =
        roast_interfaces::action::SendSmsMessage::Goal::MESSAGE_LEVEL_INFO;
  } else if (level.value() == "WARNING") {
    goal_.message_level =
        roast_interfaces::action::SendSmsMessage::Goal::MESSAGE_LEVEL_WARNING;
  } else if (level.value() == "ERROR") {
    goal_.message_level =
        roast_interfaces::action::SendSmsMessage::Goal::MESSAGE_LEVEL_ERROR;
  } else {
    throw BT::RuntimeError("invalid input [level]: ", level.value());
  }

  goal_.frequency = frequency.value();
}

BT::NodeStatus SendSmsAction::on_success() { return BT::NodeStatus::SUCCESS; }

BT::NodeStatus SendSmsAction::on_aborted() { return BT::NodeStatus::FAILURE; }

BT::NodeStatus SendSmsAction::on_cancelled() {
  // Set empty error code, action was cancelled
  return BT::NodeStatus::SUCCESS;
}

}  // namespace roast_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder = [](const std::string &name,
                               const BT::NodeConfiguration &config) {
    return std::make_unique<roast_behavior_tree::SendSmsAction>(
        name, "send_sms_message", config);
  };

  factory.registerBuilder<roast_behavior_tree::SendSmsAction>("SendSmsMessage",
                                                              builder);
}
