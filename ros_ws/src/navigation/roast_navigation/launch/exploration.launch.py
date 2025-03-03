import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory("roast_navigation"), "config", "exploration.yaml"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")

    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation/Gazebo clock"
    )
    declare_namespace_argument = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the explore node",
    )

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    exploration_node = Node(
        package="roast_exploration",
        name="explorer",
        namespace=namespace,
        executable="explore",
        parameters=[config, {"use_sim_time": use_sim_time}],
        output="screen",
        remappings=remappings,
    )

    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/mapping.launch.py"]),
    )

    # Add the nodes to the launch description
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_namespace_argument)

    ld.add_action(mapping_launch)
    ld.add_action(exploration_node)
    return ld
