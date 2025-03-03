"""Launch description for handling vision handles."""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LOG_LEVEL = os.environ.get("ROAST_LOGLEVEL", "ERROR")


def generate_launch_description():
    """Launch description with vision processing nodes."""

    ld = LaunchDescription()

    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation/Gazebo clock"
    )
    declare_namespace_argument = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    namespace = LaunchConfiguration("namespace", default="")

    hand_gesture_recognition_node = Node(
        package="roast_vision",
        executable="hand_gesture_node",
        name="hand_gesture_recognition_node",
        namespace=namespace,
        arguments=["--ros-args", "--log-level", LOG_LEVEL],
        output="both",
        respawn=True,
    )

    arucotag_detection_node = Node(
        package="roast_vision",
        executable="aruco_detection",
        name="arucotag_detection_node",
        namespace=namespace,
        arguments=["--ros-args", "--log-level", LOG_LEVEL],
        output="both",
        respawn=True,
    )

    display_photo = Node(
        package="roast_vision",
        executable="display_photo",
        name="display_photo",
        namespace=namespace,
        arguments=["--ros-args", "--log-level", LOG_LEVEL],
        output="both",
        respawn=True,
    )

    ld.add_action(declare_namespace_argument)
    ld.add_action(declare_use_sim_time_argument)

    ld.add_action(hand_gesture_recognition_node)
    ld.add_action(arucotag_detection_node)
    ld.add_action(display_photo)

    return ld
