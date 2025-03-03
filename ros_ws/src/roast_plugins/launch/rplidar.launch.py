import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

from roast.io.port_parser import PORTS

LOG_LEVEL = os.environ.get("ROAST_LOGLEVEL", "INFO")


def generate_launch_description():
    namespace = LaunchConfiguration("namespace", default="")
    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    rplidar_node = Node(
        name="rplidar_node",
        package="rplidar_ros",
        executable="rplidar_node",
        output="screen",
        namespace=namespace,
        arguments=[
            "--ros-args",
            "--log-level",
            LOG_LEVEL,
            "--enable-rosout-logs",
        ],
        parameters=[
            {
                "channel_type": "serial",
                "serial_port": PORTS["lidar"][1],
                "serial_baudrate": 115200,  # A1 / A2
                # 'serial_baudrate': 256000, # A3
                "frame_id": "laser",
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": "Standard",
            }
        ],
    )

    laser_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="laser_box_filter_chain",
        namespace=namespace,
        arguments=[
            "--ros-args",
            "--log-level",
            LOG_LEVEL,
            "--enable-rosout-logs",
        ],
        parameters=[
            PathJoinSubstitution(
                [
                    get_package_share_directory("roast_plugins"),
                    "configs",
                    "laser_filter_config.yaml",
                ]
            )
        ],
    )

    laser_time_sync_node = Node(
        package="roast_plugins",
        executable="laser_time_sync",
        name="laser_time_sync_node",
        namespace=namespace,
        arguments=["--ros-args", "--log-level", LOG_LEVEL, "--enable-rosout-logs"],
        respawn=True,
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace)

    ld.add_action(rplidar_node)
    ld.add_action(laser_filter_node)
    ld.add_action(laser_time_sync_node)

    return ld
