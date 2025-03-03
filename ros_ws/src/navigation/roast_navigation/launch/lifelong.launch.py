import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    declare_namespace_argument = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )
    return LaunchDescription(
        [
            declare_namespace_argument,
            launch_ros.actions.Node(
                parameters=[
                    get_package_share_directory("slam_toolbox")
                    + "/config/mapper_params_lifelong.yaml"
                ],
                package="slam_toolbox",
                executable="lifelong_slam_toolbox_node",
                name="lifelong_slam_toolbox_node",
                namespace=namespace,
                output="screen",
            ),
        ]
    )
