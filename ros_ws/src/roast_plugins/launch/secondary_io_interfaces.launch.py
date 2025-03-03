"""Launch sensor nodes for the roast robot."""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from roast import RobotProfile

LOG_LEVEL = os.environ.get("ROAST_LOGLEVEL", "ERROR")


def generate_launch_description():
    """Launch file for Roast robot."""
    ld = LaunchDescription()
    namespace = LaunchConfiguration("namespace", default="")
    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    environment_handler = Node(
        package="roast_plugins",
        executable="environment_state_publisher",
        name="environment_handler_node",
        namespace=namespace,
        output="screen",
        arguments=["--ros-args", "--log-level", LOG_LEVEL, "--enable-rosout-logs"],
        respawn=True,
    )

    gas_handler = Node(
        package="roast_plugins",
        executable="gas_detector",
        name="gas_handler_node",
        namespace=namespace,
        output="screen",
        arguments=["--ros-args", "--log-level", LOG_LEVEL, "--enable-rosout-logs"],
        respawn=True,
    )

    sound_alert_handler = Node(
        package="roast_plugins",
        executable="speaker",
        name="speaker_node",
        namespace=namespace,
        output="screen",
        arguments=["--ros-args", "--log-level", LOG_LEVEL, "--enable-rosout-logs"],
        respawn=True,
    )

    sms_handler = Node(
        package="roast_plugins",
        executable="sms_sender",
        name="sms_handler_node",
        namespace=namespace,
        output="screen",
        arguments=["--ros-args", "--log-level", LOG_LEVEL, "--enable-rosout-logs"],
        respawn=True,
    )

    threat_spray_module = Node(
        package="roast_plugins",
        executable="threat_spray_module",
        name="threat_spray_module",
        output="screen",
        arguments=["--ros-args", "--log-level", LOG_LEVEL, "--enable-rosout-logs"],
        respawn=True,
    )

    ld.add_action(declare_namespace)
    # ld.add_action(sms_handler)
    # ld.add_action(environment_handler)
    # ld.add_action(gas_handler)
    if RobotProfile.SPEAKER:
        ld.add_action(sound_alert_handler)
    if RobotProfile.LED_ARRAY:
        ld.add_action(threat_spray_module)

    return ld
