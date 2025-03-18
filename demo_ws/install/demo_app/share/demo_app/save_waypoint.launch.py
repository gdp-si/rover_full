import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare the filename argument
    waypoint_file = DeclareLaunchArgument(
        'waypoint_file', description='Filename for saving waypoints'
    )

    # Node for waypoint_saver with filename as a command-line argument
    waypoint_saver_node = Node(
        package='demo_app',
        executable='waypoint_saver',
        name='waypoint_saver',
        arguments=[LaunchConfiguration('waypoint_file')]
    )

    # Get the path to the robot_pose_publisher_ros2 launch file
    robot_pose_publisher_launch_file = os.path.join(
        get_package_share_directory('robot_pose_publisher_ros2'),
        'launch',
        'robot_pose_publisher_launch.py'
    )

    # Include the robot_pose_publisher launch file
    robot_pose_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_pose_publisher_launch_file)
    )

    return LaunchDescription([
        waypoint_file,
        waypoint_saver_node,
        robot_pose_publisher_launch
    ])
