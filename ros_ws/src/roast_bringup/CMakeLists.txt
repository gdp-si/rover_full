cmake_minimum_required(VERSION 3.8)
project(roast_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(roast_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)


set(dependencies
  roast_interfaces
  std_msgs
  geometry_msgs
  behaviortree_cpp_v3
)

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


# Installl executables.
# add_executable(roast src/main.cc)
# ament_target_dependencies(roast
#   ${dependencies}
# )

# install(TARGETS
#   roast
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files.
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install shell files
install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
