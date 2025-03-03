// To test the node, run the following command:
// ros2 action send_goal /patrol_action roast_interfaces/action/PatrolAction
// "{'start_patrol': 'true', 'patrol_type': 'sequence'}"

#include "roast_patrolling/patrol.hpp"

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>

#include "rclcpp_components/register_node_macro.hpp"

// TODO: Add navigate to pose with wait and pause
namespace roast_patrolling {
PatrolAction::PatrolAction(const rclcpp::NodeOptions &options)
    : Node("patrol_action_server", options) {
  using namespace std::placeholders;
  using namespace std::chrono_literals;

  declare_parameter("stop_on_failure", true);
  declare_parameter("loop_rate", 20);
  declare_parameter("map_name", "");
  package_share_directory =
      ament_index_cpp::get_package_share_directory("roast_patrolling");
  declare_parameter("file_name", "");

  // Configure
  stop_on_failure_ = get_parameter("stop_on_failure").as_bool();
  loop_rate_ = get_parameter("loop_rate").as_int();
  file_name_ = get_parameter("file_name").as_string();
  map_name_ = get_parameter("map_name").as_string();
  behavior_tree_path_ = package_share_directory + "/behavior_trees";
  with_pause_behavior_tree_path_ =
      behavior_tree_path_ + "/patroling_with_pause.xml";
  without_pause_behavior_tree_path_ =
      behavior_tree_path_ + "/patroling_without_pause.xml";

  if (file_name_ == "") {
    file_name_ = package_share_directory + "/data/" + map_name_ +
                 "/patrol_points_" + map_name_ + ".dat";
  }

  nav_to_pose_client_ = rclcpp_action::create_client<ToPoseClientT>(
      get_node_base_interface(), get_node_graph_interface(),
      get_node_logging_interface(), get_node_waitables_interface(),
      "navigate_to_pose", callback_group_);

  nav_through_poses_client_ = rclcpp_action::create_client<ThroughPoseClientT>(
      get_node_base_interface(), get_node_graph_interface(),
      get_node_logging_interface(), get_node_waitables_interface(),
      "navigate_through_poses", callback_group_);

  // Wait for navigate_to_pose action server to start
  while (!nav_to_pose_client_->wait_for_action_server(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Interrupted while waiting for the action server. Exiting.");
      return;
    }
    RCLCPP_INFO(
        this->get_logger(),
        "navigate_to_pose action server not available, waiting again...");
  }

  // Wait for navigate_through_poses action server to start
  while (!nav_through_poses_client_->wait_for_action_server(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Interrupted while waiting for the action server. Exiting.");
      return;
    }
    RCLCPP_INFO(
        this->get_logger(),
        "navigate_through_poses action server not available, waiting again...");
  }

  action_server_ = std::make_unique<ActionServer>(
      get_node_base_interface(), get_node_clock_interface(),
      get_node_logging_interface(), get_node_waitables_interface(),
      "patrol_action", std::bind(&PatrolAction::patrol, this));

  robot_pose_client_ = this->create_client<ServiceT>("get_robot_pose");

  this->getWaypointCoordinates(file_name_, waypoints_);
  // Activate
  action_server_->activate();
}

PatrolAction::~PatrolAction() { action_server_->terminate_all(); }

void PatrolAction::patrol() {
  auto goal = action_server_->get_current_goal();
  auto feedback = std::make_shared<ActionT::Feedback>();
  auto result = std::make_shared<ActionT::Result>();

  patrol_type_ = goal->patrol_type;
  patrol_mode_ = goal->patrol_mode;

  // Check if request is valid
  if (!action_server_ || !action_server_->is_server_active()) {
    RCLCPP_DEBUG(this->get_logger(), "Action Server inactive. Stopping.");
    result->success = false;
    action_server_->succeeded_current(result);
    return;
  }

  if (!goal->start_patrol) {
    RCLCPP_INFO(this->get_logger(), "Exiting since start patrol is false");
    this->cancelNavigationThroughPoses();
    this->cancelNavigationToPose();
    result->success = true;
    action_server_->succeeded_current(result);
    return;
  }

  switch (patrol_mode_) {
    case ActionT::Goal::PATROL_WITH_PAUSE:
      PatrolAction::performPatrolWithPause(goal, feedback, result);
      break;
    case ActionT::Goal::PATROL_WITHOUT_PAUSE:
      PatrolAction::performPatrolWithoutPause(goal, feedback, result);
      break;
  }

  return;
}

void PatrolAction::performPatrolWithPause(ActionT::Goal::ConstSharedPtr goal,
                                          ActionT::Feedback::SharedPtr feedback,
                                          ActionT::Result::SharedPtr result) {
  // Cancel navigation related tasks
  this->nav_to_pose_client_->async_cancel_all_goals();
  rclcpp::WallRate r(loop_rate_);
  start_patrol_ = goal->start_patrol;

  bool new_goal = true;
  bool new_intermediate_goal = true;

  std::list<geometry_msgs::msg::PoseStamped> goal_poses;
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  // Get current robot pose
  this->currentRobotPose();  // Push the coordinate to this->current_robot_pose_

  while (rclcpp::ok()) {
    // Sleep for rate
    r.sleep();

    // Check if asked to process another action
    if (action_server_->is_preempt_requested()) {
      RCLCPP_INFO(get_logger(), "Preempting the goal pose.");
      goal = action_server_->accept_pending_goal();
      patrol_type_ = goal->patrol_type;
      start_patrol_ = goal->start_patrol;
      new_goal = true;

      if (!start_patrol_) {
        RCLCPP_INFO(this->get_logger(), "Cancelling current patrol...");
        result->success = this->cancelNavigationToPose();
        action_server_->succeeded_current(result);
        return;
      }
      continue;
    }

    if (new_goal) {
      RCLCPP_INFO(this->get_logger(), "Preparing goals for patrolling");
      std::vector<geometry_msgs::msg::PoseStamped> filtered_goals =
          this->filterWaypoints(current_robot_pose_);
      if (filtered_goals.size() <= 2) {
        goal_poses = std::list<geometry_msgs::msg::PoseStamped>(
            waypoints_.begin(), waypoints_.end());
      } else {
        goal_poses = std::list<geometry_msgs::msg::PoseStamped>(
            filtered_goals.begin(), filtered_goals.end());
      }
      new_intermediate_goal = true;
      new_goal = false;
    }

    if (new_intermediate_goal) {
      ToPoseClientT::Goal goal;
      // Get the first pose
      goal.pose = goal_poses.front();
      goal_poses.pop_front();

      goal.behavior_tree = with_pause_behavior_tree_path_;
      RCLCPP_INFO(
          this->get_logger(),
          "Sending goal to navigate_to_pose server. Remaining goals: %ld",
          goal_poses.size());

      // Execute each pose to navigate_to_pose server
      auto send_goal_options =
          rclcpp_action::Client<ToPoseClientT>::SendGoalOptions();
      send_goal_options.result_callback = std::bind(
          &PatrolAction::toPoseResultCallback, this, std::placeholders::_1);
      send_goal_options.goal_response_callback =
          std::bind(&PatrolAction::toPoseGoalResponseCallback, this,
                    std::placeholders::_1);
      future_to_pose_goal_handle_ =
          nav_to_pose_client_->async_send_goal(goal, send_goal_options);
      current_goal_status_ = ActionStatus::PROCESSING;
      new_goal = false;
    }

    if (goal_poses.empty() || current_goal_status_ == ActionStatus::FAILED) {
      RCLCPP_INFO(this->get_logger(), "Closing current action request");
      new_intermediate_goal = false;
      result->success = this->cancelNavigationToPose();
      action_server_->succeeded_current(result);
      return;
    }
    if (current_goal_status_ == ActionStatus::SUCCEEDED ||
        current_goal_status_ == ActionStatus::CANCELED) {
      new_intermediate_goal = !goal_poses.empty();
      new_goal = !new_intermediate_goal;
    } else if (current_goal_status_ == ActionStatus::FAILED) {
      new_intermediate_goal = false;
      // Cancel the current goal and continue
      this->cancelNavigationToPose();
    } else {
      feedback->current_pose = current_robot_pose_;
      feedback->is_patrolling = true;
      action_server_->publish_feedback(feedback);
      new_intermediate_goal = false;
    }
  }
}

void PatrolAction::performPatrolWithoutPause(
    ActionT::Goal::ConstSharedPtr goal, ActionT::Feedback::SharedPtr feedback,
    ActionT::Result::SharedPtr result) {
  // Cancel navigation related tasks
  this->nav_through_poses_client_->async_cancel_all_goals();
  rclcpp::WallRate r(loop_rate_);
  start_patrol_ = goal->start_patrol;

  bool new_goal = true;

  RCLCPP_INFO(this->get_logger(), "Executing goal");

  // Get current robot pose
  this->currentRobotPose();  // Push the coordinate to this->current_robot_pose_

  while (rclcpp::ok()) {
    // Sleep for rate
    r.sleep();

    // Check if asked to stop processing action
    if (action_server_->is_cancel_requested()) {
      RCLCPP_INFO(this->get_logger(), "Cancelling current action request");
      bool result = this->cancelNavigationThroughPoses();
      if (result)
        current_goal_status_ = ActionStatus::CANCELED;
      else
        current_goal_status_ = ActionStatus::FAILED;
      // for result callback processing
      new_goal = false;
    }

    // Check if asked to process another action
    if (action_server_->is_preempt_requested()) {
      RCLCPP_INFO(get_logger(), "Preempting the goal pose.");
      goal = action_server_->accept_pending_goal();
      patrol_type_ = goal->patrol_type;
      start_patrol_ = goal->start_patrol;
      new_goal = true;
      continue;
    }
    if (!start_patrol_) {
      RCLCPP_INFO(this->get_logger(), "Cancelling current patrol...");
      result->success = this->cancelNavigationThroughPoses();
      action_server_->succeeded_current(result);
      return;
    }

    if (new_goal) {
      RCLCPP_INFO(this->get_logger(),
                  "Sending goal to navigate_through_pose server");
      ThroughPoseClientT::Goal goal;
      auto goal_poses = this->filterWaypoints(current_robot_pose_);
      if (goal_poses.size() <= 2) {
        goal.poses = waypoints_;
      } else {
        goal.poses = goal_poses;
      }
      goal.behavior_tree = without_pause_behavior_tree_path_;

      auto send_goal_options =
          rclcpp_action::Client<ThroughPoseClientT>::SendGoalOptions();
      send_goal_options.result_callback =
          std::bind(&PatrolAction::throughPosesResultCallback, this,
                    std::placeholders::_1);
      send_goal_options.goal_response_callback =
          std::bind(&PatrolAction::throughPosesGoalResponseCallback, this,
                    std::placeholders::_1);
      future_through_poses_goal_handle_ =
          nav_through_poses_client_->async_send_goal(goal, send_goal_options);
      current_goal_status_ = ActionStatus::PROCESSING;
      new_goal = false;
    }

    if (current_goal_status_ == ActionStatus::SUCCEEDED ||
        current_goal_status_ == ActionStatus::CANCELED) {
      new_goal = true;
      result->success = this->cancelNavigationThroughPoses();
      action_server_->succeeded_current(result);
      behavior_tree_path_ = package_share_directory + "/behavior_trees";
      return;
    } else if (current_goal_status_ == ActionStatus::FAILED) {
      result->success = false;
      this->cancelNavigationThroughPoses();
      action_server_->succeeded_current(result);
      return;
    } else {
      feedback->current_pose = current_robot_pose_;
      feedback->is_patrolling = true;
      action_server_->publish_feedback(feedback);
    }
  }
}

inline bool PatrolAction::cancelNavigationToPose() {
  auto cancel_future = this->nav_to_pose_client_->async_cancel_all_goals();
  try {
    // Spin Until complete fails since executing in a separate thread
    auto response = cancel_future.get();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Unable to cancel current action request");
    return false;
  }
  return true;
}

inline bool PatrolAction::cancelNavigationThroughPoses() {
  auto cancel_future =
      this->nav_through_poses_client_->async_cancel_all_goals();
  try {
    // Spin Until complete fails since executing in a separate thread
    auto response = cancel_future.get();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Unable to cancel current action request");
    return false;
  }
  return true;
}

void PatrolAction::getWaypointCoordinates(
    const std::string &file_name,
    std::vector<geometry_msgs::msg::PoseStamped> &waypoints) {
  RCLCPP_INFO(this->get_logger(), "Loading waypoints from %s",
              file_name.c_str());
  std::vector<std::pair<double, double>> coordinates =
      readCoordinates(file_name);

  RCLCPP_INFO(this->get_logger(), "Found %ld waypoints", coordinates.size());
  for (auto &coordinate : coordinates) {
    geometry_msgs::msg::PoseStamped waypoint;
    waypoint.header.frame_id = "map";
    waypoint.pose.position.x = coordinate.first;
    waypoint.pose.position.y = coordinate.second;
    waypoint.pose.orientation.w = 1.0;
    waypoints.push_back(waypoint);
  }
}

void PatrolAction::currentRobotPose() {
  auto pose_request = std::make_shared<ServiceT::Request>();
  pose_request->get_current_pose = true;
  auto pose_result = this->robot_pose_client_->async_send_request(pose_request);
  try {
    // Spin Until complete fails since executing in a separate thread
    auto response = pose_result.get();
    if (response->robot_pose.header.frame_id != "") {
      RCLCPP_INFO_ONCE(this->get_logger(), "Acquired current robot pose");
      current_robot_pose_ = response->robot_pose;
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Failed to get robot's current pose. Exiting current action request");
  }
}

std::vector<geometry_msgs::msg::PoseStamped> PatrolAction::filterWaypoints(
    const geometry_msgs::msg::PoseStamped &current_robot_pose) {
  std::vector<double> distances;
  for (auto point : this->waypoints_) {
    auto distance = std::sqrt(
        pow(point.pose.position.x - current_robot_pose.pose.position.x, 2) +
        pow(point.pose.position.y - current_robot_pose.pose.position.y, 2));
    distances.push_back(distance);
  }

  auto index = getIndexOfMinValue(distances);

  std::vector<geometry_msgs::msg::PoseStamped> filteredWaypoints(
      waypoints_.begin() + index, waypoints_.end());

  return filteredWaypoints;
}

inline size_t PatrolAction::getIndexOfMinValue(
    const std::vector<double> &distances) {
  auto it = std::min_element(distances.begin(), distances.end());
  return std::distance(distances.begin(), it);
}

std::vector<std::pair<double, double>> PatrolAction::readCoordinates(
    const std::string &filename) {
  std::vector<std::pair<double, double>> coordinates;
  std::ifstream file(filename);

  double x, y;

  // Skip header
  std::string header;
  std::getline(file, header);  // TODO: Check if correct order
  while (file >> x >> y) {
    coordinates.push_back(std::make_pair(x, y));
  }

  file.close();

  return coordinates;
}

void PatrolAction::toPoseResultCallback(
    const rclcpp_action::ClientGoalHandle<ToPoseClientT>::WrappedResult
        &result) {
  if (result.goal_id != future_to_pose_goal_handle_.get()->get_goal_id()) {
    RCLCPP_DEBUG(
        get_logger(),
        "Goal IDs do not match for the current goal handle and received result."
        "Ignoring likely due to receiving result for an old goal.");
    return;
  }

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      current_goal_status_ = ActionStatus::SUCCEEDED;
      return;
    case rclcpp_action::ResultCode::ABORTED:
      current_goal_status_ = ActionStatus::FAILED;
      return;
    case rclcpp_action::ResultCode::CANCELED:
      current_goal_status_ = ActionStatus::FAILED;
      return;
    default:
      current_goal_status_ = ActionStatus::UNKNOWN;
      return;
  }
}

void PatrolAction::toPoseGoalResponseCallback(
    const rclcpp_action::ClientGoalHandle<ToPoseClientT>::SharedPtr &goal) {
  if (!goal) {
    RCLCPP_ERROR(
        get_logger(),
        "navigate_to_pose action client failed to send goal to server.");
    current_goal_status_ = ActionStatus::FAILED;
  }
}

void PatrolAction::throughPosesResultCallback(
    const rclcpp_action::ClientGoalHandle<ThroughPoseClientT>::WrappedResult
        &result) {
  if (result.goal_id !=
      future_through_poses_goal_handle_.get()->get_goal_id()) {
    RCLCPP_DEBUG(
        get_logger(),
        "Goal IDs do not match for the current goal handle and received result."
        "Ignoring likely due to receiving result for an old goal.");
    return;
  }

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      current_goal_status_ = ActionStatus::SUCCEEDED;
      return;
    case rclcpp_action::ResultCode::ABORTED:
      current_goal_status_ = ActionStatus::FAILED;
      return;
    case rclcpp_action::ResultCode::CANCELED:
      current_goal_status_ = ActionStatus::FAILED;
      return;
    default:
      current_goal_status_ = ActionStatus::UNKNOWN;
      return;
  }
}

void PatrolAction::throughPosesGoalResponseCallback(
    const rclcpp_action::ClientGoalHandle<ThroughPoseClientT>::SharedPtr
        &goal) {
  if (!goal) {
    RCLCPP_ERROR(
        get_logger(),
        "navigate_through_pose action client failed to send goal to server.");
    current_goal_status_ = ActionStatus::FAILED;
  }
}
}  // namespace roast_patrolling

RCLCPP_COMPONENTS_REGISTER_NODE(roast_patrolling::PatrolAction)
