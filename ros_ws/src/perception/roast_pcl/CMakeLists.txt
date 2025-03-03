cmake_minimum_required(VERSION 3.8)
project(roast_pcl)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-pedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp_components REQUIRED)

set (dependencies
  PCL
  pcl_conversions
  sensor_msgs
  rclcpp
  rclcpp_components
)

set (libraries
  ${PCL_LIBRARIES}
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Filter nodes
add_library(roast_passthrough_filter MODULE src//filters/passthrough_filter.cc)
rclcpp_components_register_node(roast_passthrough_filter PLUGIN "roast_pcl::PassthroughFilter" EXECUTABLE passthrough_filter)

add_library(roast_cropbox_filter MODULE src//filters/cropbox_filter.cc)
rclcpp_components_register_node(roast_cropbox_filter PLUGIN "roast_pcl::CropboxFilter" EXECUTABLE cropbox_filter)

add_library(roast_voxel_grid_filter MODULE src//filters/voxel_filter.cc)
rclcpp_components_register_node(roast_voxel_grid_filter PLUGIN "roast_pcl::VoxelGridFilter" EXECUTABLE voxel_grid_filter)

list(APPEND pcl_libraries
  roast_passthrough_filter
  roast_cropbox_filter
  roast_voxel_grid_filter
)

foreach(pcl_library ${pcl_libraries})
  ament_target_dependencies(${pcl_library}
    ${dependencies}
  )
  target_link_libraries(${pcl_library}
    ${PCL_LIBRARIES}
  )
  install (TARGETS
    ${pcl_library}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endforeach()


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  configs
  DESTINATION share/${PROJECT_NAME}
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
