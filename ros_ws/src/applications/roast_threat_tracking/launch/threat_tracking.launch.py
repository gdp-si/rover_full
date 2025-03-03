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

    threat_info_publisher = Node(
        package="roast_threat_tracking",
        namespace=namespace,
        arguments=[
            "--ros-args",
            "--log-level",
            ROAST_LOGLEVEL,
            "--enable-rosout-logs",
        ],
        executable="threat_info_publisher",
        respawn=True,
    )

    threat_tracking_action_server = Node(
        package="roast_threat_tracking",
        namespace=namespace,
        arguments=[
            "--ros-args",
            "--log-level",
            ROAST_LOGLEVEL,
            "--enable-rosout-logs",
        ],
        executable="threat_tracking_action_server",
        output="screen",
        respawn=True,
    )

    ld.add_action(declare_namespace)
    ld.add_action(threat_info_publisher)
    ld.add_action(threat_tracking_action_server)

    return ld
