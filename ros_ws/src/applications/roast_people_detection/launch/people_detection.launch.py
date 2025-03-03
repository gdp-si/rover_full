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

    people_info_publisher = Node(
        package="roast_people_detection",
        namespace=namespace,
        arguments=[
            "--ros-args",
            "--log-level",
            ROAST_LOGLEVEL,
            "--enable-rosout-logs",
        ],
        executable="people_info_publisher",
        respawn=True,
    )

    move_to_people_action_server = Node(
        package="roast_people_detection",
        namespace=namespace,
        arguments=[
            "--ros-args",
            "--log-level",
            ROAST_LOGLEVEL,
            "--enable-rosout-logs",
        ],
        executable="move_to_people_action_server",
        output="screen",
        respawn=True,
    )

    ld.add_action(declare_namespace)
    ld.add_action(people_info_publisher)
    ld.add_action(move_to_people_action_server)

    return ld
