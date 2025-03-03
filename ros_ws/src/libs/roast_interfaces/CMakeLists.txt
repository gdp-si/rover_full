cmake_minimum_required(VERSION 3.5)
project(roast_interfaces)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EncoderData.msg"
  "msg/Vision.msg"
  "msg/Diagnostics.msg"
  "msg/Robot.msg"
  "msg/ThreatInfo.msg"
  "msg/ThreatGroupInfo.msg"
  "msg/LedMode.msg"
  "msg/HandGesture.msg"
  "msg/PeopleGroup.msg"
  "msg/DisplayMode.msg"

  # AI Module
  "msg/ObjectTracking.msg"

  "srv/Fan.srv"
  "srv/JetsonClocks.srv"
  "srv/NVPModel.srv"
  "srv/GetRobotPose.srv"
  "srv/AlertMode.srv"
  "srv/DetectAruco.srv"
  "srv/CaptureCamera.srv"
  "srv/WaitForHandGesture.srv"

  # Hardware IO
  "srv/SprayModule.srv"
  "srv/LedModeSelector.srv"
  "srv/Display.srv"
  "srv/DisplayPhoto.srv"

  # Diagnostics
  "srv/DeleteLogs.srv"
  "srv/ConfigLogger.srv"

  "action/GoToPose.action"
  "action/TrackTarget.action"
  "action/ConnectToChargingStation.action"
  "action/SendCommandCenterMessage.action"
  "action/SendSmsMessage.action"
  "action/SpeakerAnnouncement.action"
  "action/MoveToPeople.action"

  # Application actions
  "action/PatrolAction.action"
  "action/MoveForward.action"
  "action/GoToHome.action"
  DEPENDENCIES builtin_interfaces sensor_msgs geometry_msgs nav_msgs std_msgs stereo_msgs diagnostic_msgs
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

ament_package()
