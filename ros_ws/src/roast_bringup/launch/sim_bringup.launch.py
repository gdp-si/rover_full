"""Spawn and launch the roast robot."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

LOG_LEVEL = os.environ.get("ROAST_LOGLEVEL", "ERROR")

navigation_dir = get_package_share_directory("roast_navigation")
this_dir = get_package_share_directory("roast_bringup")
pcl_dir = get_package_share_directory("roast_pcl")
vision_dir = get_package_share_directory("roast_vision")
services_dir = get_package_share_directory("roast_services")
actions_dir = get_package_share_directory("roast_actions")
patrolling_dir = get_package_share_directory("roast_patrolling")
threat_tracking_dir = get_package_share_directory("roast_threat_tracking")
behavior_tree_dir = get_package_share_directory("roast_behavior_tree")
rosbridge_dir = get_package_share_directory("rosbridge_server")


remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

bootup_file_name = "bootup_service.sh"
bootup_config = os.path.join(this_dir, "scripts", bootup_file_name)
way_point_file_name = "waypoint.sh"
way_point_config = os.path.join(this_dir, "scripts", way_point_file_name)

progress_bar_name = "sensor_stats.py"
progress_bar_path = os.path.join(this_dir, "scripts", progress_bar_name)

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
    "localization": 0.0,
    "pcl": 12.0,
    "vision": 12.0,
    "navigation": 0.0,  # 15.0,
    "services_and_actions": 30.0,
    "patrolling": 40.0,
    "threat_tracking": 40.0,
    "behavior_trees": 60.0,
}


def generate_launch_description():
    """Launch configurations for pointcloud to laserscan conversion."""
    use_sim_time = "True"

    namespace = LaunchConfiguration("namespace")
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    static_map_path = os.path.join(
        navigation_dir, "maps", f"{str(map_configuration)}.yaml"
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

    pcl_launch = TimerAction(
        period=WAIT_PERIOD["pcl"],
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([pcl_dir, "/launch/pcl.launch.py"]),
                launch_arguments=[
                    ("namespace", namespace),
                ],
            )
        ],
    )

    vision_launch = TimerAction(
        period=WAIT_PERIOD["vision"],
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([vision_dir, "/launch/vision.launch.py"]),
                launch_arguments=[
                    ("namespace", namespace),
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
                    ("use_sim_time", use_sim_time),
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
                    ("use_sim_time", use_sim_time),
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
                launch_arguments=[
                    ("namespace", namespace),
                    ("use_sim_time", use_sim_time),
                ],
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
                launch_arguments=[
                    ("namespace", namespace),
                    ("use_sim_time", use_sim_time),
                ],
            )
        ],
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([this_dir, "/launch/rviz.launch.py"]),
        launch_arguments=[
            ("namespace", namespace),
            ("use_sim_time", use_sim_time),
        ],
    )

    rosbridge_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            [rosbridge_dir, "/launch/rosbridge_websocket_launch.xml"]
        )
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    # ld.add_action(localization_launch)
    ld.add_action(pcl_launch)
    ld.add_action(vision_launch)
    ld.add_action(navigation_launch)
    ld.add_action(services_launch)
    ld.add_action(actions_launch)
    ld.add_action(patrolling_launch)
    # ld.add_action(threat_tracking_launch)
    # ld.add_action(behavior_tree_launch)
    ld.add_action(rviz_launch)
    ld.add_action(rosbridge_launch)

    return ld
