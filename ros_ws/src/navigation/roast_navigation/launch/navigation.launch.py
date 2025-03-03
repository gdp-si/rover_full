"""Navigation launch file for skid steering robot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from roast_navigation.libs.parameter_substitution import update_navigation_params

from roast import RobotProfile
from roast.utils import robot_initial_pose

LOG_LEVEL = os.environ.get("ROAST_LOGLEVEL", "ERROR")
ROBOT_CONFIGS = RobotProfile.get_configurations()


bringup_dir = get_package_share_directory("roast_navigation")
remappings = [
    ("/odom", "odometry/filtered"),
    ("/tf", "tf"),
    ("/tf_static", "tf_static"),
]
static_map_path = None
default_behavior_tree = os.path.join(
    bringup_dir, "behavior_trees", "navigate_w_replanning_and_recovery.xml"
)
navigation_params_template = os.path.join(bringup_dir, "config", "navigation.yaml")
navigation_filter_params_template = os.path.join(
    bringup_dir, "config", "nav2_filters.yaml"
)


def launch_setup(context, *args, **kwargs):  # pylint: disable=unused-argument
    """Launch configurations for navigation setup."""
    # Create the launch configuration variables
    global static_map_path

    namespace = LaunchConfiguration("namespace").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time", default=False).perform(context)
    map_configuration = LaunchConfiguration(
        "map_configuration", default="office_space"
    ).perform(context)
    static_map_path = os.path.join(
        bringup_dir, "maps", f"{str(map_configuration)}.yaml"
    )

    params_file = LaunchConfiguration(
        "params_file", default=navigation_params_template
    ).perform(context)
    autostart = LaunchConfiguration("autostart", default=True).perform(context)

    param_substitutions = {
        "use_sim_time": use_sim_time,
        "autostart": autostart,
        "default_nav_to_pose_bt_xml": default_behavior_tree,
    }

    configured_params = update_navigation_params(
        params_file=params_file,
        param_callback=rewritten_navigation_params,
        param_substitutions=param_substitutions,
        namespace=str(namespace),
    )

    nav2_filter_params = update_navigation_params(
        params_file=navigation_filter_params_template,
        param_callback=rewritten_navigation_params,
        param_substitutions=param_substitutions,
        namespace=str(namespace),
    )

    nav2_controller = Node(
        package="nav2_controller",
        executable="controller_server",
        namespace=namespace,
        output="screen",
        respawn=True,
        parameters=[configured_params],
        remappings=remappings,
        arguments=["--ros-args", "--log-level", LOG_LEVEL, "--enable-rosout-logs"],
    )

    nav2_planner = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        namespace=namespace,
        respawn=True,
        output="screen",
        parameters=[configured_params],
        remappings=remappings,
        arguments=["--ros-args", "--log-level", LOG_LEVEL, "--enable-rosout-logs"],
    )

    nav2_behaviors = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        namespace=namespace,
        respawn=True,
        output="screen",
        parameters=[configured_params],
        remappings=remappings,
        arguments=["--ros-args", "--log-level", LOG_LEVEL, "--enable-rosout-logs"],
    )

    nav2_bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        namespace=namespace,
        respawn=True,
        output="screen",
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", LOG_LEVEL, "--enable-rosout-logs"],
        remappings=remappings,
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        namespace=namespace,
        respawn=True,
        output="screen",
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", LOG_LEVEL, "--enable-rosout-logs"],
    )

    nav2_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        namespace=namespace,
        respawn=True,
        output="screen",
        parameters=[configured_params],
        remappings=remappings,
        arguments=["--ros-args", "--log-level", LOG_LEVEL, "--enable-rosout-logs"],
    )

    nav2_map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver",
        namespace=namespace,
        respawn=True,
        output="screen",
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", LOG_LEVEL, "--enable-rosout-logs"],
    )

    # Localization
    nav2_amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        namespace=namespace,
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", LOG_LEVEL, "--enable-rosout-logs"],
        remappings=remappings,
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        namespace=namespace,
        remappings=remappings,
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "map",
            "odom",
        ],
        output="screen",
        respawn=True,
    )

    initial_pose_handler = Node(
        package="roast_navigation",
        executable="initial_pose_handler",
        name="initial_pose_handler",
        namespace=namespace,
        remappings=remappings,
        arguments=["--ros-args", "--log-level", LOG_LEVEL, "--enable-rosout-logs"],
        output="screen",
        respawn=True,
    )

    odom_pose_handler = Node(
        package="roast_navigation",
        executable="odom_pose_handler",
        name="odom_pose_handler",
        namespace=namespace,
        arguments=["--ros-args", "--log-level", LOG_LEVEL, "--enable-rosout-logs"],
        remappings=remappings,
        output="screen",
        respawn=True,
    )

    node_names = [
        "initial_pose_handler",
        "odom_pose_handler",
    ]

    roast_lifecycle_manager = Node(
        package="roast_lifecycle_manager",
        executable="lifecycle_manager",
        parameters=[
            {"autostart": True},
            {"node_names": node_names, "no_of_retries": 1},
        ],
        name="roast_nav2_lifecycle_manager",
        namespace=namespace,
        arguments=["--ros-args", "--log-level", LOG_LEVEL, "--enable-rosout-logs"],
        respawn=True,
    )

    filter_mask_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="keepout_mask_server",
        namespace=namespace,
        respawn=True,
        output="screen",
        parameters=[nav2_filter_params],
        remappings=remappings,
        arguments=["--ros-args", "--log-level", LOG_LEVEL, "--enable-rosout-logs"],
    )

    keepout_costmap_filter = Node(
        package="nav2_map_server",
        executable="costmap_filter_info_server",
        name="keepout_filter_info_server",
        namespace=namespace,
        output="screen",
        respawn=True,
        parameters=[nav2_filter_params],
        remappings=remappings,
        arguments=["--ros-args", "--log-level", LOG_LEVEL, "--enable-rosout-logs"],
    )

    speedzone_mask_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="speedzone_mask_server",
        namespace=namespace,
        output="screen",
        respawn=True,
        parameters=[nav2_filter_params],
        remappings=remappings,
        arguments=["--ros-args", "--log-level", LOG_LEVEL, "--enable-rosout-logs"],
    )

    speedzone_costmap_filter = Node(
        package="nav2_map_server",
        executable="costmap_filter_info_server",
        name="speedzone_filter_info_server",
        namespace=namespace,
        output="screen",
        respawn=True,
        parameters=[nav2_filter_params],
        remappings=remappings,
        arguments=["--ros-args", "--log-level", LOG_LEVEL, "--enable-rosout-logs"],
    )

    launch_descriptions = [
        nav2_map_saver,
        nav2_map_server,
        nav2_controller,
        nav2_planner,
        nav2_behaviors,
        nav2_bt_navigator,
        nav2_lifecycle_manager,
        initial_pose_handler,
        odom_pose_handler,
        roast_lifecycle_manager,
    ]

    if ROBOT_CONFIGS["RP_LIDAR"] or ROBOT_CONFIGS["VELODYNE"]:
        launch_descriptions.append(nav2_amcl_node)
    else:
        launch_descriptions.append(static_tf)

    if ROBOT_CONFIGS["NAVIGATION_KEEPOUT_ZONES"]:
        launch_descriptions.append(filter_mask_server)
        launch_descriptions.append(keepout_costmap_filter)

    if ROBOT_CONFIGS["NAVIGATION_SPEED_LIMITS"]:
        launch_descriptions.append(speedzone_mask_server)
        launch_descriptions.append(speedzone_costmap_filter)

    return launch_descriptions


def generate_launch_description():
    """Generate launch description with multiple components."""
    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_map_configuration_cmd = DeclareLaunchArgument(
        "map_configuration",
        default_value="",
        description="Map name for setting the configurations",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false" if RobotProfile != "SIM" else "true",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=navigation_params_template,
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_configuration_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld


# FIXME: Namespaced robots are not launching properly
# due to a currently unknown bug in lifecycle_manager
# As a survey, no multi-robot launch files are currently using lifecycle manger,
# so this may not be a blocker
# fmt: off
def rewritten_navigation_params(namespace) -> dict:
    """Rewritten paramaters."""
    rewritten_params = {}
    namespaced_params = {}

    scan_topic = ""
    if ROBOT_CONFIGS["VELODYNE"] and ROBOT_CONFIGS["RP_LIDAR"]:
        scan_topic = "/velodyne/scan/filtered"
    elif ROBOT_CONFIGS["VELODYNE"]:
        scan_topic = "/velodyne/scan/filtered"
    elif ROBOT_CONFIGS["RP_LIDAR"]:
        scan_topic = "/scan/filtered"

    # COMMON PARAMETERS
    rewritten_params.update(
        {
            "odom_topic": "odometry/filtered",
            "costmap_topic": "/local_costmap/costmap_raw",
            "footprint_topic": "/local_costmap/published_footprint",
            "scan_topic": scan_topic if scan_topic != "" else "/scan",
        }
    )

    # AMCL
    x, y, z, yaw = robot_initial_pose(with_quaternion=False)
    rewritten_params.update(
        {
            "amcl.ros__parameters.initial_pose.x": str(x),
            "amcl.ros__parameters.initial_pose.y": str(y),
            "amcl.ros__parameters.initial_pose.z": str(z),
            "amcl.ros__parameters.initial_pose.yaw": str(yaw),
        }
    )

    # MAP SERVER
    rewritten_params.update(
        {
            "map_server.ros__parameters.yaml_filename": static_map_path,
        }
    )

    # KEEPOUT MASK SERVER
    keepout_mask_path = os.path.join(
        os.path.dirname(static_map_path),
        static_map_path.split(".")[-2] + "_keepout_mask.yaml",
    )

    rewritten_params.update(
        {
            "keepout_mask_server.ros__parameters.yaml_filename": keepout_mask_path,
        }
    )

    # SPEEDZONE MASK SERVER
    speedzone_mask_path = os.path.join(
        os.path.dirname(static_map_path),
        static_map_path.split(".")[-2] + "_speedzone_mask.yaml",
    )

    rewritten_params.update(
        {
            "speedzone_mask_server.ros__parameters.yaml_filename": speedzone_mask_path,
        }
    )

    # LOCAL COSTMAP
    params = {
        "local_costmap.local_costmap.ros__parameters.obstacle_layer.enabled": str(ROBOT_CONFIGS["RP_LIDAR"]),
        "local_costmap.local_costmap.ros__parameters.velodyne_obstacle_layer.enabled": str(ROBOT_CONFIGS["VELODYNE"]),
        # "local_costmap.local_costmap.ros__parameters.voxel_layer.enabled": str(ROBOT_CONFIGS["VELODYNE"]),
        "local_costmap.local_costmap.ros__parameters.range_layer.enabled": str(ROBOT_CONFIGS["TOF_SENSOR_ARRAY"]),
        "local_costmap.local_costmap.ros__parameters.keepout_filter.enabled": str(ROBOT_CONFIGS["NAVIGATION_KEEPOUT_ZONES"]),
        "local_costmap.local_costmap.ros__parameters.speedzone_filter.enabled": str(ROBOT_CONFIGS["NAVIGATION_SPEED_LIMITS"]),
        "local_costmap.local_costmap.ros__parameters.obstacle_layer.scan_source.topic": "/scan/filtered",
        "local_costmap.local_costmap.ros__parameters.velodyne_obstacle_layer.scan_source.topic": "/velodyne/scan/filtered",
        # "local_costmap.local_costmap.ros__parameters.voxel_layer.voxel_layer.topic": "/velodyne_points/radius/filtered",
    }

    if ROBOT_CONFIGS["TOF_SENSOR_ARRAY_LIST"]:
        params["local_costmap.local_costmap.ros__parameters.range_layer.topics"] = ROBOT_CONFIGS["TOF_SENSOR_ARRAY_LIST"]

    rewritten_params.update(params)

    # GLOBAL COSTMAP
    params = {
        "global_costmap.global_costmap.ros__parameters.keepout_filter.enabled": str(ROBOT_CONFIGS["NAVIGATION_KEEPOUT_ZONES"]),
        "global_costmap.global_costmap.ros__parameters.speedzone_filter.enabled": str(ROBOT_CONFIGS["NAVIGATION_SPEED_LIMITS"]),
        "global_costmap.global_costmap.ros__parameters.obstacle_layer.enabled": str(ROBOT_CONFIGS["RP_LIDAR"]),
        "global_costmap.global_costmap.ros__parameters.velodyne_obstacle_layer.enabled": str(ROBOT_CONFIGS["VELODYNE"]),
        # "global_costmap.global_costmap.ros__parameters.voxel_layer.enabled": str(ROBOT_CONFIGS["VELODYNE"]),
        "global_costmap.global_costmap.ros__parameters.range_layer.enabled": str(ROBOT_CONFIGS["TOF_SENSOR_ARRAY"]),
        "global_costmap.global_costmap.ros__parameters.obstacle_layer.scan_source.topic": "/scan/filtered",
        "global_costmap.global_costmap.ros__parameters.velodyne_obstacle_layer.scan_source.topic": "/velodyne/scan/filtered",
        # "global_costmap.global_costmap.ros__parameters.voxel_layer.voxel_layer.topic": "/velodyne_points/radius/filtered",
    }
    rewritten_params.update(params)

    if namespace != "":
        for param, value in rewritten_params.items():
            namespaced_params[param] = f"/{namespace}{value}"

    # LIFECYCLE MANAGER
    lifecycle_nodes = {
        "node_names": [
            "map_server",
            "map_saver",
            "controller_server",
            "planner_server",
            "behavior_server",
            "bt_navigator",
        ],
    }

    # Update lifecycle nodes based on robot configuration
    # Fix the order of nodes
    if ROBOT_CONFIGS["VELODYNE"] or ROBOT_CONFIGS["RP_LIDAR"]:
        lifecycle_nodes["node_names"].insert(2, "amcl")
    if ROBOT_CONFIGS["NAVIGATION_KEEPOUT_ZONES"]:
        lifecycle_nodes["node_names"].insert(3, "keepout_mask_server")
        lifecycle_nodes["node_names"].insert(4, "keepout_filter_info_server")
    if ROBOT_CONFIGS["NAVIGATION_SPEED_LIMITS"]:
        lifecycle_nodes["node_names"].insert(5, "speedzone_mask_server")
        lifecycle_nodes["node_names"].insert(6, "speedzone_filter_info_server")

    if namespace != "":
        for param, value in lifecycle_nodes.items():
            namespaced_params[param] = [f"/{namespace}/{v}" for v in value]
    else:
        rewritten_params.update(lifecycle_nodes)

    if namespace != "":
        return namespaced_params
    else:
        return rewritten_params
