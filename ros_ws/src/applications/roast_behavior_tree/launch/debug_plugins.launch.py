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

    send_command_center_message = Node(
        package="roast_behavior_tree",
        namespace=namespace,
        arguments=[
            "--ros-args",
            "--log-level",
            ROAST_LOGLEVEL,
            "--enable-rosout-logs",
        ],
        executable="SendCommandCenterMessageDebugNode",
    )

    speaker_announcement_debug_node = Node(
        package="roast_behavior_tree",
        namespace=namespace,
        arguments=[
            "--ros-args",
            "--log-level",
            ROAST_LOGLEVEL,
            "--enable-rosout-logs",
        ],
        executable="SpeakerAnnouncementDebugNode",
    )

    send_sms_message_debug_node = Node(
        package="roast_behavior_tree",
        namespace=namespace,
        arguments=[
            "--ros-args",
            "--log-level",
            ROAST_LOGLEVEL,
            "--enable-rosout-logs",
        ],
        executable="SendSmsMessageDebugNode",
    )

    activate_led_service_debug_node = Node(
        package="roast_behavior_tree",
        namespace=namespace,
        arguments=[
            "--ros-args",
            "--log-level",
            ROAST_LOGLEVEL,
            "--enable-rosout-logs",
        ],
        executable="ActivateLedServiceDebugNode",
    )

    threat_mitigation_service_debug_node = Node(
        package="roast_behavior_tree",
        namespace=namespace,
        arguments=[
            "--ros-args",
            "--log-level",
            ROAST_LOGLEVEL,
            "--enable-rosout-logs",
        ],
        executable="ThreatMitigationServiceDebugNode",
    )

    patrol_action_debug_node = Node(
        package="roast_behavior_tree",
        namespace=namespace,
        arguments=[
            "--ros-args",
            "--log-level",
            ROAST_LOGLEVEL,
            "--enable-rosout-logs",
        ],
        executable="PatrolActionDebugNode",
    )
    track_threat_debug_node = Node(
        package="roast_behavior_tree",
        namespace=namespace,
        arguments=[
            "--ros-args",
            "--log-level",
            ROAST_LOGLEVEL,
            "--enable-rosout-logs",
        ],
        executable="TrackThreatDebugNode",
    )

    ld.add_action(declare_namespace)
    ld.add_action(send_command_center_message)
    ld.add_action(speaker_announcement_debug_node)
    ld.add_action(send_sms_message_debug_node)
    ld.add_action(activate_led_service_debug_node)
    ld.add_action(threat_mitigation_service_debug_node)
    ld.add_action(patrol_action_debug_node)
    ld.add_action(track_threat_debug_node)

    return ld
