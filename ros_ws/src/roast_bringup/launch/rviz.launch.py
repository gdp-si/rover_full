import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

pkg_dir = FindPackageShare(package="roast_bringup").find("roast_bringup")
rviz_config = os.path.join(pkg_dir, "rviz", "view_rviz.rviz")


def generate_launch_description():
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    namespace = LaunchConfiguration("namespace", default="")

    # Create the rviz node
    rviz_entity = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=namespace,
        remappings=remappings,
        arguments=["-d", rviz_config],
    )

    ld = LaunchDescription()
    ld.add_action(rviz_entity)

    return ld
