cmake_minimum_required(VERSION 3.8)
project(roast_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(roast_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(
  include
)

set(library_name ${PROJECT_NAME})

set(dependencies
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  sensor_msgs
  behaviortree_cpp_v3
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  nav2_msgs
  nav2_behavior_tree
  nav2_util
  roast_interfaces
  rclcpp_components
  cv_bridge
)

# Action Nodes
add_library(SendCommandCenterMessage SHARED src/actions/send_command_center_message.cc)
list(APPEND plugin_libs SendCommandCenterMessage)

add_library(MoveForward SHARED src/actions/move_forward.cc)
list(APPEND plugin_libs MoveForward)

add_library(PatrolAction SHARED src/actions/patrol.cc)
list(APPEND plugin_libs PatrolAction)

add_library(CancelPatrolAction SHARED src/actions/patrol_cancel.cc)
list(APPEND plugin_libs CancelPatrolAction)

add_library(TrackThreat SHARED src/actions/track_threat.cc)
list(APPEND plugin_libs TrackThreat)

add_library(SendSmsMessage SHARED src/actions/send_sms_message.cc)
list(APPEND plugin_libs SendSmsMessage)

add_library(SpeakerAnnouncement SHARED src/actions/speaker_announcement.cc)
list(APPEND plugin_libs SpeakerAnnouncement)

add_library(CancelTrackThreat SHARED src/actions/track_threat_cancel.cc)
list(APPEND plugin_libs CancelTrackThreat)

add_library(ThreatMitigationService SHARED src/actions/threat_mitigation.cc)
list(APPEND plugin_libs ThreatMitigationService)

add_library(ActivateLedService SHARED src/actions/activate_led.cc)
list(APPEND plugin_libs ActivateLedService)

add_library(GoToHome SHARED src/actions/goto_home.cc)
list(APPEND plugin_libs GoToHome)

add_library(ClearCostmapService SHARED src/actions/clear_costmap_service.cc)
list(APPEND plugin_libs ClearCostmapService)

add_library(RecoveryActions SHARED src/actions/recovery_actions.cc)
list(APPEND plugin_libs RecoveryActions)

add_library(CaptureCamera SHARED src/actions/capture_camera.cc)
list(APPEND plugin_libs CaptureCamera)

add_library(MoveToPeopleGroup SHARED src/actions/move_to_people.cc)
list(APPEND plugin_libs MoveToPeopleGroup)

add_library(WaitForHandGesture SHARED src/actions/wait_for_gesture.cc)
list(APPEND plugin_libs WaitForHandGesture)

add_library(DisplayMode SHARED src/actions/display_mode.cc)
list(APPEND plugin_libs DisplayMode)

add_library(DisplayPhoto SHARED src/actions/display_photo.cc)
list(APPEND plugin_libs DisplayPhoto)

add_library(DeleteHistoryService SHARED src/actions/delete_history.cc)
list(APPEND plugin_libs DeleteHistoryService)

# Condition Nodes
add_library(ThreatDetectionConditions SHARED src/conditions/threat_detection_conditions.cc)
list(APPEND plugin_libs ThreatDetectionConditions)

add_library(AreSensorsReady SHARED src/conditions/are_sensors_ready.cc)
list(APPEND plugin_libs AreSensorsReady)

add_library(IsArucoDetected SHARED src/conditions/is_aruco_detected.cc)
list(APPEND plugin_libs IsArucoDetected)

add_library(IsHandGestureDetected SHARED src/conditions/is_hand_gesture.cc)
list(APPEND plugin_libs IsHandGestureDetected)

add_library(IsGroupDetected SHARED src/conditions/is_group_detected.cc)
list(APPEND plugin_libs IsGroupDetected)

# Debug Action Nodes
add_library(SendCommandCenterMessageDebug SHARED debug_nodes/send_command_center_message_debug.cc)
list(APPEND debug_nodes SendCommandCenterMessageDebug)

add_library(PatrolActionDebug SHARED debug_nodes/patrol_action_debug.cc)
list(APPEND debug_nodes PatrolActionDebug)

add_library(TrackThreatDebug SHARED debug_nodes/track_threat_debug.cc)
list(APPEND debug_nodes TrackThreatDebug)

add_library(SpeakerAnnouncementDebug SHARED debug_nodes/speaker_announcement_debug.cc)
list(APPEND debug_nodes SpeakerAnnouncementDebug)

add_library(SendSmsMessageDebug SHARED debug_nodes/send_sms_message_debug.cc)
list(APPEND debug_nodes SendSmsMessageDebug)

# Debug Service Nodes
add_executable(ThreatMitigationServiceDebugNode debug_nodes/threat_mitigation_service_debug.cc)
list(APPEND debug_service_nodes ThreatMitigationServiceDebugNode)

add_executable(ActivateLedServiceDebugNode debug_nodes/activate_led_service_debug.cc)
list(APPEND debug_service_nodes ActivateLedServiceDebugNode)

# Install Plugins
foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin}
    ${dependencies}
  )
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(
  TARGETS ${plugin_libs}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

# Install Debug Action Nodes
foreach(debug_node ${debug_nodes})
  target_include_directories(${debug_node} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  ament_target_dependencies(${debug_node}
    ${dependencies}
  )
  target_compile_definitions(${debug_node} PRIVATE "ROAST_BEHAVIOR_TREE_EXPORT")
  rclcpp_components_register_node(${debug_node}
    PLUGIN "roast_behavior_tree::${debug_node}"
    EXECUTABLE ${debug_node}Node)

endforeach()

# Install Debug Service Nodes
foreach(debug_node ${debug_service_nodes})
  ament_target_dependencies(${debug_node} ${dependencies})
  install(TARGETS
    ${debug_node}
    DESTINATION lib/${PROJECT_NAME})
endforeach()

install(
  TARGETS ${debug_nodes}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${plugin_libs}
  ${debug_nodes}
)

ament_export_dependencies(
  ${dependencies}
)

add_executable(roast_bt_engine src/bt_engine.cc)
ament_target_dependencies(roast_bt_engine
  ${dependencies}
)

add_executable(roast_subscriber src/misc/subscriber.cc)
ament_target_dependencies(roast_subscriber
  ${dependencies}
)

install(TARGETS
  roast_bt_engine
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  roast_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  configs
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
