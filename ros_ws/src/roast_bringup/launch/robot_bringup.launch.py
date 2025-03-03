"""Spawn and launch the roast robot."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from roast import RobotProfile

LOG_LEVEL = os.environ.get("ROAST_LOGLEVEL", "ERROR")

navigation_dir = get_package_share_directory("roast_navigation")
plugins_dir = get_package_share_directory("roast_plugins")
pcl_dir = get_package_share_directory("roast_pcl")
vision_dir = get_package_share_directory("roast_vision")
this_dir = get_package_share_directory("roast_bringup")
services_dir = get_package_share_directory("roast_services")
actions_dir = get_package_share_directory("roast_actions")
patrolling_dir = get_package_share_directory("roast_patrolling")
threat_tracking_dir = get_package_share_directory("roast_threat_tracking")
behavior_tree_dir = get_package_share_directory("roast_behavior_tree")
rosbridge_dir = get_package_share_directory("rosbridge_server")
diagnostics_dir = get_package_share_directory("roast_diagnostics")


remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
rviz_file_name = "view_rviz.rviz"
rviz_config = os.path.join(this_dir, "rviz", rviz_file_name)

bootup_file_name = "bootup_service.sh"
bootup_config = os.path.join(this_dir, "scripts", bootup_file_name)
way_point_file_name = "waypoint.sh"
way_point_config = os.path.join(this_dir, "scripts", way_point_file_name)

progress_bar_name = "sensor_stats.py"
progress_bar_path = os.path.join(this_dir, "scripts", progress_bar_name)

ros_bag_record = os.environ.get("ROS_BAG_RECORD", None)

# Get the map configuration from environment
map_configuration = os.environ.get("MAP_CONFIGURATION", None)
if map_configuration is None:
    raise ValueError("Please set the map configuration in environment")

map_dirs = os.path.join(navigation_dir, "maps")
map_names = []
for file in os.listdir(map_dirs):
    map_names.append(file.split(".")[0])

if map_configuration.lower() not in map_names:
    raise ValueError(
        f"Map configuration {map_configuration} is not found in {map_dirs}"
    )

WAIT_PERIOD = {
    "sensors": 5.0,
    "pcl": 12.0,
    "vision": 12.0,
    "localization": 15.0,
    "navigation": 30.0,
    "services_and_actions": 60.0,
    "patrolling": 70.0,
    "threat_tracking": 75.0,
    "diagnostics": 80.0,
    "record": 85.0,
    "behavior_trees": 90.0,
    "way_point": 90.0,
}


def generate_launch_description():
    """Launch configurations for pointcloud to laserscan conversion."""
    namespace = LaunchConfiguration("namespace")
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    boot_up_service = ExecuteProcess(cmd=[["/bin/bash ", bootup_config]], shell=True)

    static_map_path = os.path.join(
        navigation_dir, "maps", f"{str(map_configuration)}.yaml"
    )

    launch_file = [plugins_dir, "/launch/robot_io_interfaces.launch.py"]

    setup_launch = TimerAction(
        period=WAIT_PERIOD["sensors"],
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file),
                launch_arguments=[("namespace", namespace)],
            )
        ],
    )

    pcl_launch = TimerAction(
        period=WAIT_PERIOD["pcl"],
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([pcl_dir, "/launch/pcl.launch.py"]),
                launch_arguments=[("namespace", namespace)],
            )
        ],
    )

    vision_launch = TimerAction(
        period=WAIT_PERIOD["vision"],
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([vision_dir, "/launch/vision.launch.py"]),
                launch_arguments=[("namespace", namespace)],
            )
        ],
    )

    localization_launch = TimerAction(
        period=WAIT_PERIOD["localization"],
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [navigation_dir, "/launch/robot_localization.launch.py"]
                ),
                launch_arguments=[
                    ("namespace", namespace),
                    ("map", static_map_path),
                ],
            )
        ],
    )

    navigation_launch = TimerAction(
        period=WAIT_PERIOD["navigation"],
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [navigation_dir, "/launch/navigation.launch.py"]
                ),
                launch_arguments=[
                    ("namespace", namespace),
                    ("use_sim_time", "False"),
                    ("map_configuration", map_configuration),
                ],
            )
        ],
    )

    services_launch = TimerAction(
        period=WAIT_PERIOD["services_and_actions"],
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [services_dir, "/launch/services.launch.py"]
                ),
                launch_arguments=[("namespace", namespace)],
            )
        ],
    )

    actions_launch = TimerAction(
        period=WAIT_PERIOD["services_and_actions"],
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [actions_dir, "/launch/action_servers.launch.py"]
                ),
                launch_arguments=[("namespace", namespace)],
            ),
        ],
    )
    patrolling_launch = TimerAction(
        period=WAIT_PERIOD["patrolling"],
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [patrolling_dir, "/launch/patrolling.launch.py"]
                ),
                launch_arguments=(
                    ("namespace", namespace),
                    ("use_sim_time", "False"),
                    ("map_configuration", map_configuration),
                ),
            )
        ],
    )

    threat_tracking_launch = TimerAction(
        period=WAIT_PERIOD["threat_tracking"],
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [threat_tracking_dir, "/launch/threat_tracking.launch.py"]
                ),
                launch_arguments=[("namespace", namespace), ("use_sim_time", "False")],
            )
        ],
    )

    behavior_tree_launch = TimerAction(
        period=WAIT_PERIOD["behavior_trees"],
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [behavior_tree_dir, "/launch/behavior_tree_engine.launch.py"]
                ),
                launch_arguments=[("namespace", namespace), ("use_sim_time", "False")],
            )
        ],
    )

    rosbridge_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            [rosbridge_dir, "/launch/rosbridge_websocket_launch.xml"]
        )
    )

    diagnostics_launch = TimerAction(
        period=WAIT_PERIOD["diagnostics"],
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [diagnostics_dir, "/launch/diagnostics.launch.py"]
                ),
            )
        ],
    )

    record_launch = TimerAction(
        period=WAIT_PERIOD["record"],
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [this_dir, "/launch/record_data.launch.py"]
                ),
            )
        ],
    )

    ld = LaunchDescription()
    ld.add_action(boot_up_service)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(setup_launch)
    ld.add_action(pcl_launch)
    ld.add_action(vision_launch)
    ld.add_action(localization_launch)
    ld.add_action(navigation_launch)
    ld.add_action(services_launch)
    ld.add_action(actions_launch)
    ld.add_action(patrolling_launch)
    ld.add_action(rosbridge_launch)
    # ld.add_action(diagnostics_launch)

    if ros_bag_record == "1":
        ld.add_action(record_launch)

    # if RobotProfile.INTEL_REALSENSE or RobotProfile.OAKD_ARRAY:
    #     ld.add_action(threat_tracking_launch)

    ld.add_action(behavior_tree_launch)

    return ld
