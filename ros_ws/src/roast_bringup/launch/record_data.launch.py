import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

from roast import RobotProfile

LOG_LEVEL = os.environ.get("ROAST_LOGLEVEL", "ERROR")

this_dir = get_package_share_directory("roast_bringup")


def generate_launch_description():
    ROBOT_CONFIGS = RobotProfile.get_configurations()

    topics_list = []
    topics_list.append("/encoder_left/data")
    topics_list.append("/encoder_right/data")
    topics_list.append("/odometry/motor_odom")
    topics_list.append("/cmd_vel")
    if ROBOT_CONFIGS["BNO055"] or ROBOT_CONFIGS["ARDUINO_IMU"]:
        topics_list.append("/imu")

    topics_list.append("/device_stats")

    topics_list.append("/amcl_pose")
    topics_list.append("/odometry/filtered")
    topics_list.append("/plan")
    # if ROBOT_CONFIGS["LIDAR_SLAM"]:
    #     topics_list.append("/current_pose")
    #     topics_list.append("/lidar_scan")
    #     topics_list.append("/map")
    #     topics_list.append("/path")

    if ROBOT_CONFIGS["OAKD_ARRAY"]:
        topics_list.append("/threat_info")

    # Bag file name and path
    date_prefix = datetime.now().strftime("%d_%m_%Y")
    timestamp_prefix = datetime.now().strftime("%H_%M_%s")
    bag_file_name = f"{timestamp_prefix}_recorded_data"

    bag_file_path = os.path.join(
        os.path.expanduser("~"), ".roast", "logs", date_prefix, bag_file_name
    )

    node = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-s", "mcap", "-d", "3600", "-o", bag_file_path]
        + topics_list,
        name="record_all_topics",
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(node)
    return ld
