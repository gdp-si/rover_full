"""Launch description for handling diagnostics."""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LOG_LEVEL = os.environ.get("ROAST_LOGLEVEL", "ERROR")


def generate_launch_description():
    """Launch description with diagnostics processing nodes."""

    ld = LaunchDescription()

    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation/Gazebo clock"
    )
    declare_namespace_argument = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    namespace = LaunchConfiguration("namespace", default="")

    stats_node = Node(
        package="roast_diagnostics",
        executable="device_stats",
        name="stats_node",
        namespace=namespace,
        arguments=["--ros-args", "--log-level", LOG_LEVEL],
        output="both",
        respawn=True,
    )

    node_names = [
        "stats_node",
    ]

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

    ld.add_action(declare_namespace_argument)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(stats_node)
    ld.add_action(lifecycle_manager)

    return ld
