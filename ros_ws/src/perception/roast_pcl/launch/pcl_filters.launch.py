"""Launch description for handling point cloud filtering and processing."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from roast import RobotProfile

LOG_LEVEL = os.environ.get("ROAST_LOGLEVEL", "ERROR")

default_filters_parameters = os.path.join(
    get_package_share_directory("roast_pcl"),
    "configs",
    "filter_params.yaml",
)


def generate_launch_description():
    """Launch file for GMapping SLAM with Slam toolbox."""

    ld = LaunchDescription()

    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation/Gazebo clock"
    )
    decalare_filters_parameters = DeclareLaunchArgument(
        "filters_param_file",
        default_value=default_filters_parameters,
        description="Absolute path to the ROS2 parameters file to use for filter nodes",
    )
    declare_namespace_argument = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    namespace = LaunchConfiguration("namespace", default="")

    voxel_grid_filter_node = Node(
        package="roast_pcl",
        executable="voxel_grid_filter",
        name="voxel_grid_filter_node",
        remappings=[
            ("input", "velodyne_points"),
            ("output", "velodyne_points/downsampled"),
        ],
        parameters=[default_filters_parameters],
        namespace=namespace,
        output="screen",
        respawn=True,
    )

    ground_plane_segmentation_node = Node(
        package="roast_pcl",
        executable="passthrough_filter",
        name="ground_plane_segmentation_node",
        remappings=[
            ("input", "velodyne_points/downsampled"),
            ("output", "velodyne_points/ground_removed"),
        ],
        parameters=[default_filters_parameters],
        namespace=namespace,
        output="screen",
        respawn=True,
    )

    robot_footprint_filter_node = Node(
        package="roast_pcl",
        executable="cropbox_filter",
        name="robot_footprint_filter_node",
        remappings=[
            ("input", "velodyne_points/ground_removed"),
            ("output", "velodyne_points/filtered"),
        ],
        parameters=[default_filters_parameters],
        namespace=namespace,
        output="screen",
        respawn=True,
    )

    radius_filter_node = Node(
        package="roast_pcl",
        executable="cropbox_filter",
        name="radius_filter_node",
        remappings=[
            ("input", "velodyne_points/filtered"),
            ("output", "velodyne_points/radius/filtered"),
        ],
        parameters=[default_filters_parameters],
        namespace=namespace,
        output="screen",
        respawn=True,
    )

    ld.add_action(declare_namespace_argument)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(decalare_filters_parameters)

    if RobotProfile.VELODYNE:
        ld.add_action(voxel_grid_filter_node)
        ld.add_action(ground_plane_segmentation_node)
        ld.add_action(robot_footprint_filter_node)
        ld.add_action(radius_filter_node)

    return ld
