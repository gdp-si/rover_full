"""Spawn and launch the roast robot."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node

LOG_LEVEL = os.environ.get("ROAST_LOGLEVEL", "ERROR")

plugins_dir = get_package_share_directory("roast_plugins")
this_dir = get_package_share_directory("roast_slam")


remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
rviz_file_name = "mapping.rviz"
rviz_config = os.path.join(this_dir, "rviz", rviz_file_name)

WAIT_PERIOD = {
    "realsense": 5.0,
    "rtabmap": 10.0,
}


def generate_launch_description():
    """Launch configurations for pointcloud to laserscan conversion."""
    namespace = LaunchConfiguration("namespace")
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    realsense_launch = TimerAction(
        period=WAIT_PERIOD["realsense"],
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [plugins_dir, "/launch/realsense.launch.py"]
                ),
                launch_arguments=[
                    ("namespace", namespace),
                ],
            )
        ],
    )

    rtabmap_launch = TimerAction(
        period=WAIT_PERIOD["rtabmap"],
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([this_dir, "/launch/rtabmap.launch.py"]),
                launch_arguments=[
                    ("namespace", namespace),
                ],
            )
        ],
    )

    rviz_entity = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=namespace,
        remappings=remappings,
        arguments=["-d", rviz_config],
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(realsense_launch)
    ld.add_action(rtabmap_launch)
    # ld.add_action(rviz_entity)

    return ld
