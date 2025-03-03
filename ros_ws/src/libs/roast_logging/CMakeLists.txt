cmake_minimum_required(VERSION 3.8)
project(roast_logging)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(roast_interfaces REQUIRED)

include_directories(include)

set(dependencies
  "rclcpp"
  "rclcpp_components"
  "rcutils"
  "std_msgs"
  "roast_interfaces"
)
add_library(roast_logging_component SHARED
  src/logger_config_component.cc
)
target_compile_definitions(roast_logging_component
  PRIVATE "ROAST_LOGGING_BUILDING_DLL")
ament_target_dependencies(roast_logging_component
  ${dependencies}
)
rclcpp_components_register_nodes(roast_logging_component "roast_logging::RoastLoggingComponent")

add_library(roast_usage_component SHARED
  src/logger_usage_component.cc
)
target_compile_definitions(roast_usage_component
  PRIVATE "ROAST_LOGGING_BUILDING_DLL"
)
ament_target_dependencies(roast_usage_component
  ${dependencies}
)
rclcpp_components_register_nodes(roast_usage_component "roast_logging::LoggerConfig")

install(TARGETS
  roast_logging_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

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
