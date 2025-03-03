"""Velodyne lidar launch file"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir

from roast import RobotProfile

LOG_LEVEL = os.environ.get("ROAST_LOGLEVEL", "ERROR")


def generate_launch_description():
    """Launch file for velodyne VLP16"""
    ld = LaunchDescription()
    namespace = LaunchConfiguration("namespace", default="")
    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    velodyne_io_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/velodyne.launch.py"]),
        launch_arguments=[("namespace", namespace)],
    )

    velodyne_filters_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/pcl_filters.launch.py"]),
        launch_arguments=[("namespace", namespace)],
    )

    velodyne_points_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ThisLaunchFileDir(), "/pointcloud_to_laserscan.launch.py"]
        ),
        launch_arguments=[("namespace", namespace)],
    )

    ld.add_action(declare_namespace)

    if RobotProfile.VELODYNE:
        if not RobotProfile.SIM:
            ld.add_action(velodyne_io_launch)
    ld.add_action(velodyne_filters_launch)
    ld.add_action(velodyne_points_launch)

    return ld
