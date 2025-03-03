"""Launch sensor nodes for the roast robot."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node

from roast import RobotProfile

LOG_LEVEL = os.environ.get("ROAST_LOGLEVEL", "ERROR")


def generate_launch_description():
    """Launch file for Roast robot."""
    ld = LaunchDescription()
    ROBOT_CONFIGS = RobotProfile.get_configurations()

    namespace = LaunchConfiguration("namespace", default="")
    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    # Include velodyne launch file
    rplidar_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/rplidar.launch.py"]),
        launch_arguments=[("namespace", namespace)],
    )

    realsense_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/realsense.launch.py"]),
        launch_arguments=[("namespace", namespace)],
    )

    secondary_sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ThisLaunchFileDir(), "/secondary_io_interfaces.launch.py"]
        ),
        launch_arguments=[("namespace", namespace)],
    )

    battery_node = Node(
        package="roast_plugins",
        executable="battery",
        name="battery_state_publisher",
        namespace=namespace,
        arguments=["--ros-args", "--log-level", LOG_LEVEL, "--enable-rosout-logs"],
        respawn=True,
    )

    imu_node = Node(
        package="roast_plugins",
        executable="imu",
        name="imu_node",
        namespace=namespace,
        arguments=["--ros-args", "--log-level", LOG_LEVEL, "--enable-rosout-logs"],
        respawn=True,
    )

    camera_node = Node(
        package="roast_plugins",
        executable="oakd_array",
        name="oakd_array_node",
        namespace=namespace,
        arguments=["--ros-args", "--log-level", LOG_LEVEL, "--enable-rosout-logs"],
        respawn=True,
    )

    odom_node = Node(
        package="roast_plugins",
        executable="odometry",
        name="odom_node",
        namespace=namespace,
        parameters=[
            {"odometry_type": "motor"},  # OPTIONS: lidar, motor
            {"odom_target_frame": "base_footprint"},
        ],
        arguments=["--ros-args", "--log-level", LOG_LEVEL, "--enable-rosout-logs"],
        respawn=True,
    )

    range_node = Node(
        package="roast_plugins",
        executable="range",
        name="range_sensor_array_node",
        namespace=namespace,
        output="screen",
        respawn=True,
    )

    ultrasonic_node = Node(
        package="roast_plugins",
        executable="ultrasonic",
        namespace=namespace,
        output="screen",
        respawn=True,
    )

    robot_control_node = Node(
        package="roast_plugins",
        executable="robot_control",
        name="robot_control_node",
        namespace=namespace,
        arguments=["--ros-args", "--log-level", LOG_LEVEL, "--enable-rosout-logs"],
        respawn=True,
    )

    # Utilities

    robot_state_publisher_node = Node(
        package="roast_plugins",
        executable="robot_state_publisher",
        name="robot_state_publisher_node",
        namespace=namespace,
        arguments=["--ros-args", "--log-level", LOG_LEVEL, "--enable-rosout-logs"],
        respawn=True,
    )

    display_node = Node(
        package="roast_plugins",
        executable="display_node",
        name="display_node",
        namespace=namespace,
        output="screen",
        respawn=True,
    )

    # Default nodes for all robots
    ld.add_action(declare_namespace)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(robot_control_node)
    ld.add_action(secondary_sensors)
    ld.add_action(odom_node)
    # ld.add_action(display_node)

    # TODO: Check if this order is working as expected
    node_names = [
        "robot_state_publisher_node",
        "robot_control_node",
        "odom_node",
        # "display_node",
    ]

    # Setup launch description based on robot profile
    if ROBOT_CONFIGS["BNO055"] or ROBOT_CONFIGS["ARDUINO_IMU"]:
        ld.add_action(imu_node)
        node_names.append("imu_node")
    if ROBOT_CONFIGS["OAKD_ARRAY"]:
        ld.add_action(camera_node)
        node_names.append("oakd_array_node")
    if ROBOT_CONFIGS["RP_LIDAR"]:
        ld.add_action(rplidar_launch_file)
    if ROBOT_CONFIGS["TOF_SENSOR_ARRAY"]:
        ld.add_action(range_node)
        node_names.append("range_sensor_array_node")
    if ROBOT_CONFIGS["ULTRASONIC_SENSOR_ARRAY"]:
        ld.add_action(ultrasonic_node)
        node_names.append("ultrasonic_node")
    if ROBOT_CONFIGS["INTEL_REALSENSE"]:
        ld.add_action(realsense_launch_file)
    if ROBOT_CONFIGS["LED_ARRAY"]:
        node_names.append("threat_spray_module")
    if ROBOT_CONFIGS["SPEAKER"]:
        node_names.append("speaker_node")

    lifecycle_manager = Node(
        package="roast_lifecycle_manager",
        executable="lifecycle_manager",
        parameters=[
            {"autostart": True},
            {"node_names": node_names, "no_of_retries": 5},
        ],
        name="roast_lifecycle_manager",
        namespace=namespace,
        arguments=["--ros-args", "--log-level", LOG_LEVEL, "--enable-rosout-logs"],
        respawn=False,
    )

    ld.add_action(lifecycle_manager)

    return ld
