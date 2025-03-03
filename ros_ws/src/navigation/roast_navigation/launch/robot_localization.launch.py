import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

LOG_LEVEL = os.environ.get("ROAST_LOGLEVEL", "ERROR")


def generate_launch_description():
    """
    Generate launch description for ROS2 package.

    Returns
    -------
    list
        List of tasks to do during launch

    """

    pkg_share = FindPackageShare(package="roast_navigation").find("roast_navigation")

    namespace = LaunchConfiguration("namespace")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        namespace=namespace,
        parameters=[
            os.path.join(pkg_share, "config/ekf.yaml"),
            {"use_sim_time": False},
        ],
        respawn=True,
        arguments=["--ros-args", "--log-level", LOG_LEVEL, "--enable-rosout-logs"],
    )

    return LaunchDescription([declare_namespace_cmd, robot_localization_node])
