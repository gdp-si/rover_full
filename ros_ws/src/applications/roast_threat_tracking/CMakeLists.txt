cmake_minimum_required(VERSION 3.8)
project(roast_threat_tracking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(roast_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

set (dependencies
  rclcpp
  rclpy
  ament_cmake
  ament_cmake_python
  rclcpp_action
  roast_interfaces
  rclcpp_components
  nav2_msgs
  nav_msgs
  nav2_util
  geometry_msgs
  visualization_msgs
)

include_directories(include)

add_library(roast_threat_tracking MODULE
  src/threat_tracking.cc
  )
rclcpp_components_register_node(roast_threat_tracking PLUGIN "roast_threat_tracking::ThreatTracker" EXECUTABLE threat_tracking_action_server)

target_compile_definitions(roast_threat_tracking
  PRIVATE "ROAST_THREAT_TRACKING_CPP_BUILDING_DLL")

ament_target_dependencies(roast_threat_tracking ${dependencies})

install(PROGRAMS python/threat_info_publisher DESTINATION lib/roast_threat_tracking)
install(PROGRAMS python/debug_threat_tracking DESTINATION lib/roast_threat_tracking)
install (PROGRAMS python/oakd_threat_info_publisher DESTINATION lib/roast_threat_tracking)

install(TARGETS
  roast_threat_tracking
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(DIRECTORY
  behavior_trees
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})


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

ament_package()
