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

    move_forward_action_server = Node(
        package="roast_actions",
        namespace=namespace,
        arguments=[
            "--ros-args",
            "--log-level",
            ROAST_LOGLEVEL,
            "--enable-rosout-logs",
        ],
        executable="move_forward_action_server",
        output="screen",
    )

    send_sms_action_server = Node(
        package="roast_actions",
        namespace=namespace,
        arguments=[
            "--ros-args",
            "--log-level",
            ROAST_LOGLEVEL,
            "--enable-rosout-logs",
        ],
        executable="send_sms_action_server",
        output="screen",
    )

    speaker_announcement_action_server = Node(
        package="roast_actions",
        namespace=namespace,
        arguments=[
            "--ros-args",
            "--log-level",
            ROAST_LOGLEVEL,
            "--enable-rosout-logs",
        ],
        executable="speaker_announcement_action_server",
        output="screen",
    )

    goto_home_action_server = Node(
        package="roast_actions",
        namespace=namespace,
        arguments=[
            "--ros-args",
            "--log-level",
            ROAST_LOGLEVEL,
            "--enable-rosout-logs",
        ],
        executable="goto_home_action_server",
        output="screen",
    )

    ld.add_action(declare_namespace)
    ld.add_action(move_forward_action_server)
    ld.add_action(send_sms_action_server)
    ld.add_action(speaker_announcement_action_server)
    ld.add_action(goto_home_action_server)

    return ld
