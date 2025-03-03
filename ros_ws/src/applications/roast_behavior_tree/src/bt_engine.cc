#include "roast_behavior_tree/bt_engine.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <csignal>
#include <filesystem>

#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"

// TODO (selva): Extend bt node actions to return BT::NodeStatus::RUNNING
// TODO (selva): Add a new node to the tree to publish the blackboard
// TODO (selva): Add service to write the manifest file (applicable BT.CPP 4)

namespace roast_behavior_tree {
namespace fs = std::filesystem;

BtEngine::BtEngine() : Node("bt_engine") {
  configure_parameters();
  load_plugins();
  register_subtrees();
  load_tree();
  run();
}

void BtEngine::configure_parameters() {
  RCLCPP_INFO(this->get_logger(), "Configuring behavior parameters");
  std::string package_path =
      ament_index_cpp::get_package_share_directory("roast_behavior_tree");
  std::string bt_xml_file = package_path + "/behavior_trees/main.xml";
  bt_file_path_ = this->declare_parameter("bt_file_path", bt_xml_file);
  loop_timeout_ =
      std::chrono::milliseconds(this->declare_parameter("loop_timeout", 100));
  std::vector<std::string> default_plugins_ = {"SendCommandCenterMessage"};
  plugins_ = this->declare_parameter("behavior_plugins", default_plugins_);

  // Groot
  run_groot_monitoring_ = this->declare_parameter("run_groot_monitoring", true);
  publisher_port_ = this->declare_parameter("publisher_port", 1668);
  server_port_ = this->declare_parameter("server_port", 1669);
  max_msg_per_second_ = this->declare_parameter("max_msg_per_second", 25);
  tree_name_ = this->declare_parameter("tree_name", "MainTree");
}

void BtEngine::register_subtrees() {
  std::string package_path =
      ament_index_cpp::get_package_share_directory("roast_behavior_tree");
  std::string bt_xml_directory = package_path + "/behavior_trees/subtrees/";
  // Walk through directory and load all files
  if (fs::exists(bt_xml_directory) && fs::is_directory(bt_xml_directory)) {
    for (const auto &entry : fs::directory_iterator(bt_xml_directory)) {
      auto file_basename = entry.path().filename().string();
      auto msg = "Registering subtree from file: " + file_basename;
      RCLCPP_INFO(this->get_logger(), msg.c_str());
      factory_.registerBehaviorTreeFromFile(entry.path().string());
    }
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "Error loading Subtrees. Directory %s does not exist or is "
                 "not a directory",
                 bt_xml_directory.c_str());
  }
}

void BtEngine::load_tree() {
  auto file_basename = fs::path(bt_file_path_).filename().string();
  auto msg = "Registering main tree from file: " + file_basename;
  RCLCPP_INFO(this->get_logger(), msg.c_str());
  factory_.registerBehaviorTreeFromFile(bt_file_path_);
}

void BtEngine::run() {
  rclcpp::WallRate loop_rate(loop_timeout_);

  // Setup blackboard variables for the main tree
  auto blackboard = Blackboard::create();
  blackboard->set<rclcpp::Node::SharedPtr>(
      "node", std::make_shared<rclcpp::Node>("bt_node"));
  blackboard->set<std::chrono::milliseconds>("bt_loop_duration", loop_timeout_);
  blackboard->set<std::chrono::milliseconds>("server_timeout",
                                             std::chrono::milliseconds(3000));
  //  blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_);

  // Check if tree name is available in the factory
  std::vector<std::string> tree_names = factory_.registeredBehaviorTrees();
  RCLCPP_INFO(this->get_logger(), "Checking if %s is available in the factory",
              tree_name_.c_str());
  if (std::find(tree_names.begin(), tree_names.end(), tree_name_) ==
      tree_names.end()) {
    RCLCPP_ERROR(this->get_logger(), "Tree name %s not found in the factory",
                 tree_name_.c_str());
    BT::RuntimeError("Tree name not found in the factory");
  }

  // Create tree
  tree_ = std::make_shared<Tree>(factory_.createTree(tree_name_, blackboard));
  StdCoutLogger logger_cout(*tree_);

  if (run_groot_monitoring_) {
    add_groot_monitoring();
  }
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Running tree at frequency "
                         << 1.0 / loop_timeout_.count() * 1e3 << "Hz");
  while (rclcpp::ok()) {
    BT::NodeStatus result = tree_->tickRootWhileRunning();
    if (result == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(this->get_logger(),
                  "Successfully completed one iteration of the tree");
    } else {
      RCLCPP_FATAL(this->get_logger(),
                   "Tree failed without completing one iteration");
    }
    loop_rate.sleep();
  }
}

void BtEngine::add_groot_monitoring() {
  RCLCPP_INFO(this->get_logger(),
              "Groot monitoring enabled with server port [ %d ] and publisher "
              "port [ %d ]",
              server_port_, publisher_port_);
  groot_monitor_ = std::make_unique<BT::PublisherZMQ>(
      *tree_, max_msg_per_second_, publisher_port_, server_port_);
}

void BtEngine::load_plugins() {
  RCLCPP_INFO(this->get_logger(), "Loading plugins");
  RCLCPP_INFO(this->get_logger(), "Number of plugins: %ld", plugins_.size());
  for (const auto &p : plugins_) {
    auto msg = "Loading plugin: " + SharedLibrary::getOSName(p);
    RCLCPP_DEBUG(this->get_logger(), msg.c_str());
    factory_.registerFromPlugin(SharedLibrary::getOSName(p));
  }
}

void sigint_handler(
    __attribute__((unused)) int signal_num) {  // Silences compiler warnings
  rclcpp::shutdown();
  exit(0);
}
}  // namespace roast_behavior_tree

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  signal(SIGINT, roast_behavior_tree::sigint_handler);
  rclcpp::spin(std::make_shared<roast_behavior_tree::BtEngine>());
  rclcpp::shutdown();
  return 0;
}
