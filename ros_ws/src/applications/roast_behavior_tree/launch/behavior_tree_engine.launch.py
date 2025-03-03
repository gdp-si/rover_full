import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node

ROAST_LOGLEVEL = os.environ.get("ROAST_LOGLEVEL", "ERROR")


def read_configs():
    config_file = os.path.join(
        get_package_share_directory("roast_behavior_tree"), "configs/bt_engine.yaml"
    )
    with open(config_file, "r") as f:
        configs = yaml.safe_load(f)

    return configs


def generate_launch_description():
    ld = LaunchDescription()

    namespace = LaunchConfiguration("namespace", default="")
    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    debug_mode = LaunchConfiguration(
        "debug_mode", default="true"
    )  # TODO: change to false
    declare_debug_mode = DeclareLaunchArgument(
        "debug_mode", default_value="false", description="Debug mode"
    )

    tree_path = os.path.join(
        get_package_share_directory("roast_behavior_tree"),
        "behavior_trees/main.xml",
    )
    declare_tree_path = DeclareLaunchArgument(
        "tree_path",
        default_value=tree_path,
        description="Absolute path to the behavior tree file",
    )

    declare_tree_name = DeclareLaunchArgument(
        "tree_name",
        default_value="PatrollingSubTree",
        description="Name of the behavior Tree ID to execute as main tree",
    )

    debug_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ThisLaunchFileDir(), "/debug_plugins.launch.py"]
        ),
        launch_arguments=[("namespace", namespace)],
        condition=IfCondition(debug_mode),
    )

    configs = read_configs()

    bt_engine = Node(
        package="roast_behavior_tree",
        namespace=namespace,
        arguments=[
            "--ros-args",
            "--log-level",
            ROAST_LOGLEVEL,
            "--enable-rosout-logs",
        ],
        executable="roast_bt_engine",
        # Do not declare a node name otherwise it messes with the action node names and will result in
        # duplicate nodes!
        parameters=[
            {"bt_file_path": LaunchConfiguration("tree_path")},
            {"behavior_plugins": configs["behavior_plugins"]},
            {"tree_name": LaunchConfiguration("tree_name")},
        ],
        output="screen",
        respawn=True,
    )

    ld.add_action(declare_debug_mode)
    ld.add_action(debug_launch)
    ld.add_action(declare_tree_path)
    ld.add_action(declare_tree_name)
    ld.add_action(declare_namespace)
    ld.add_action(bt_engine)

    return ld
