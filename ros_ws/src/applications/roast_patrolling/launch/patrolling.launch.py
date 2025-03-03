import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROAST_LOGLEVEL = os.environ.get("ROAST_LOGLEVEL", "ERROR")


def generate_launch_description():
    ld = LaunchDescription()

    namespace = LaunchConfiguration("namespace", default="")
    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )
    map_configuration = LaunchConfiguration("map_configuration", default="office_space")
    declare_map_configuration = DeclareLaunchArgument(
        "map_configuration",
        default_value="",
        description="Map Name to choose the waypoints from",
    )

    patrolling_action_server = Node(
        package="roast_patrolling",
        namespace=namespace,
        arguments=[
            "--ros-args",
            "--log-level",
            ROAST_LOGLEVEL,
            "--enable-rosout-logs",
        ],
        executable="patrolling_action_server",
        output="screen",
        parameters=[{"map_name": map_configuration}],
        respawn=True,
    )

    ld.add_action(declare_namespace)
    ld.add_action(declare_map_configuration)
    ld.add_action(patrolling_action_server)

    return ld
