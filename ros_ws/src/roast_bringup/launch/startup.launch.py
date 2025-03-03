import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess

from roast import RobotProfile

LOG_LEVEL = os.environ.get("ROAST_LOGLEVEL", "ERROR")

this_dir = get_package_share_directory("roast_bringup")

username = os.getlogin()

# Log file name and path
date_prefix = datetime.now().strftime("%d_%m_%Y")
timestamp_prefix = datetime.now().strftime("%H_%M_%s")
log_file_name = f"{timestamp_prefix}.log"

log_file_path = os.path.join(
    os.path.expanduser("~"), ".roast", "logs", date_prefix, log_file_name
)

# Create the necessary directories if they don't exist
os.makedirs(os.path.dirname(log_file_path), exist_ok=True)


def generate_launch_description():
    launch_command = (
        f"ros2 launch roast_bringup robot_bringup.launch.py | tee -a {log_file_path}"
    )

    bringup = ExecuteProcess(
        cmd=["/bin/sh", "-c", launch_command],
        name="bringup",
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(bringup)
    return ld
