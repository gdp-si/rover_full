cmake_minimum_required(VERSION 3.8)
project(roast_people_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(roast_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set (dependencies
  rclcpp
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

add_library(roast_people_detection MODULE
  src/move_to_people.cc
)

rclcpp_components_register_node(roast_people_detection PLUGIN "roast_people_detection::MoveToPeopleActionServer" EXECUTABLE move_to_people_action_server)

target_compile_definitions(roast_people_detection
  PRIVATE "ROAST_PEOPLE_DETECTION_CPP_BUILDING_DLL")

ament_target_dependencies(roast_people_detection ${dependencies})

install(PROGRAMS python/people_info_publisher DESTINATION lib/roast_people_detection)
install(DIRECTORY launch DESTINATION share/roast_people_detection)
install(DIRECTORY behavior_trees DESTINATION share/roast_people_detection)

install(TARGETS
  roast_people_detection
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
