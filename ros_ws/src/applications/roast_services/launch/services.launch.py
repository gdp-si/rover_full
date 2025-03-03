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

    config_params = [
        {
            "odometry_topic": "odometry/filtered",
        }
    ]

    get_robot_pose_service = Node(
        package="roast_services",
        namespace=namespace,
        arguments=[
            "--ros-args",
            "--log-level",
            ROAST_LOGLEVEL,
            "--enable-rosout-logs",
        ],
        executable="get_robot_pose_service",
        parameters=config_params,
        respawn=True,
    )

    led_mode_selector_service = Node(
        package="roast_services",
        namespace=namespace,
        arguments=[
            "--ros-args",
            "--log-level",
            ROAST_LOGLEVEL,
            "--enable-rosout-logs",
        ],
        executable="led_mode_selector_service",
        parameters=config_params,
        respawn=True,
    )

    display_mode_selector_service = Node(
        package="roast_services",
        namespace=namespace,
        arguments=[
            "--ros-args",
            "--log-level",
            ROAST_LOGLEVEL,
            "--enable-rosout-logs",
        ],
        executable="display_mode_selector_service",
        parameters=config_params,
        respawn=True,
    )

    set_alert_mode_service = Node(
        package="roast_services",
        namespace=namespace,
        arguments=[
            "--ros-args",
            "--log-level",
            ROAST_LOGLEVEL,
            "--enable-rosout-logs",
        ],
        executable="set_alert_mode_service",
        parameters=config_params,
        respawn=True,
    )

    capture_camera_service = Node(
        package="roast_services",
        namespace=namespace,
        arguments=[
            "--ros-args",
            "--log-level",
            ROAST_LOGLEVEL,
            "--enable-rosout-logs",
        ],
        executable="capture_camera_service",
        parameters=config_params,
        respawn=True,
    )

    ld.add_action(declare_namespace)
    ld.add_action(get_robot_pose_service)
    ld.add_action(set_alert_mode_service)
    ld.add_action(led_mode_selector_service)
    ld.add_action(capture_camera_service)
    ld.add_action(display_mode_selector_service)

    return ld
