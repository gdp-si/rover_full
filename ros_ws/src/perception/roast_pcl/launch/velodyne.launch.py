"""Velodyne lidar launch file"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

LOG_LEVEL = os.environ.get("ROAST_LOGLEVEL", "ERROR")


def generate_launch_description():
    """Launch file for velodyne VLP16"""
    ld = LaunchDescription()
    namespace = LaunchConfiguration("namespace", default="")
    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    shared_pkg = get_package_share_directory("roast_plugins")
    driver_params = os.path.join(shared_pkg, "configs", "velodyne-vlp16.yaml")
    velodyne_driver_node = Node(
        package="velodyne_driver",
        executable="velodyne_driver_node",
        name="velodyne_driver_node",
        namespace=namespace,
        parameters=[driver_params],
        output="both",
        arguments=["--ros-args", "--log-level", LOG_LEVEL, "--enable-rosout-logs"],
    )

    convert_params = os.path.join(
        shared_pkg, "configs", "velodyne-pointcloud-vlp16.yaml"
    )
    with open(convert_params, "r", encoding="utf-8") as f:
        params = yaml.safe_load(f)["velodyne_transform_node"]["ros__parameters"]
    params["calibration"] = os.path.join(
        shared_pkg, "configs", "velodyne-vlp16-params.yaml"
    )

    velodyne_pointcloud = ComposableNodeContainer(
        name="velodyne_pointcloud_convert_container",
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container",
        arguments=["--ros-args", "--log-level", LOG_LEVEL, "--enable-rosout-logs"],
        composable_node_descriptions=[
            ComposableNode(
                package="velodyne_pointcloud",
                plugin="velodyne_pointcloud::Transform",
                name="velodyne_transform_node",
                parameters=[params],
            ),
        ],
        output="both",
    )

    ld.add_action(declare_namespace)

    ld.add_action(velodyne_driver_node)
    ld.add_action(velodyne_pointcloud)

    return ld
