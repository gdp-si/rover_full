cmake_minimum_required(VERSION 3.8)
project(roast_services)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(roast_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

set (dependencies
  rclcpp
  roast_interfaces
  nav2_msgs
  nav_msgs
  nav2_util
  sensor_msgs
  cv_bridge
  image_transport
  )

add_executable(get_robot_pose_service src/get_robot_pose.cc)
list(APPEND services get_robot_pose_service)

add_executable(delete_log_service src/delete_logs.cc)
list(APPEND services delete_log_service)

add_executable(set_alert_mode_service src/set_alert_mode.cc)
list(APPEND services set_alert_mode_service)

add_executable(led_mode_selector_service src/led_mode_selector.cc)
list(APPEND services led_mode_selector_service)

add_executable(capture_camera_service src/capture_camera_service.cc)
list(APPEND services capture_camera_service)

add_executable(display_mode_selector_service src/display_mode_selector.cc)
list(APPEND services display_mode_selector_service)

foreach(service ${services})
  ament_target_dependencies(${service} ${dependencies})
endforeach()

install(TARGETS
  ${services}
  DESTINATION lib/${PROJECT_NAME})

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
