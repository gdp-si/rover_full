cmake_minimum_required(VERSION 3.8)
project(roast_patrolling)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(roast_interfaces REQUIRED)
find_package(nav2_util REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

set (dependencies
  rclcpp
  rclcpp_action
  roast_interfaces
  rclcpp_components
  nav2_msgs
  nav_msgs
  nav2_util
  geometry_msgs
)

include_directories(include)

add_library(roast_patrolling MODULE
  src/patrol.cc
  )
rclcpp_components_register_node(roast_patrolling PLUGIN "roast_patrolling::PatrolAction" EXECUTABLE patrolling_action_server)

target_compile_definitions(roast_patrolling
  PRIVATE "ROAST_PATROLLING_CPP_BUILDING_DLL")

ament_target_dependencies(roast_patrolling ${dependencies})

install(TARGETS
  roast_patrolling
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(DIRECTORY
  behavior_trees
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  data
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
