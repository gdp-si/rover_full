import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LOG_LEVEL = os.environ.get("ROAST_LOGLEVEL", "ERROR")


def generate_launch_description():
    """Launch configurations for pointcloud to laserscan conversion."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="namespace",
                default_value="",
                description="Namespace for sample topics",
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                remappings=[
                    (
                        "cloud_in",
                        [
                            LaunchConfiguration(variable_name="namespace"),
                            "velodyne_points/radius/filtered",
                        ],
                    ),
                    (
                        "scan",
                        [
                            LaunchConfiguration(variable_name="namespace"),
                            "velodyne/scan/filtered",
                        ],
                    ),
                ],
                arguments=["--ros-args", "--log-level", LOG_LEVEL],
                parameters=[
                    {
                        "target_frame": "velodyne",
                        "transform_tolerance": 0.01,
                        "min_height": 0.0,
                        "max_height": 1.0,
                        "angle_min": -3.141519,  # -2 * M_PI/2
                        "angle_max": 3.141519,  # 2 * M_PI/2
                        "angle_increment": 0.0087,  # M_PI/360.0
                        "scan_time": 0.0,
                        "range_min": 1.0,
                        "range_max": 100.0,
                        "use_inf": False,
                        "inf_epsilon": 1.0,
                    }
                ],
                name="pointcloud_to_laserscan",
            ),
        ]
    )
