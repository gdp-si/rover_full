import os

from roast.settings import DATA_PATH
from roast.utils.math import quaternion_to_euler


def robot_current_pose(with_quaternion: bool = True) -> tuple:
    """Get the current pose of the robot."""
    # TODO: Introduce lock to avoid concurrency issues
    ROBOT_POSE_FILE = os.path.join(os.path.join(DATA_PATH, "odometry.dat"))
    if os.path.exists(ROBOT_POSE_FILE):
        with open(ROBOT_POSE_FILE, "r") as f:
            file = f.read()
            data = file.split(" ")

            # remove few elements from the list
            x, y, z = float(data[3]), float(data[4]), 0.0
            qx, qy, qz, qw = (
                float(data[6]),
                float(data[7]),
                float(data[8]),
                float(data[9]),
            )

            if with_quaternion:
                return x, y, z, qx, qy, qz, qw

            _, _, yaw = quaternion_to_euler((qx, qy, qz, qw))

            return x, y, z, yaw
    else:
        if with_quaternion:
            return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0

        return 0.0, 0.0, 0.0, 0.0


def robot_initial_pose(with_quaternion: bool = True) -> tuple:
    """Get the current pose of the robot."""
    # TODO: Introduce lock to avoid concurrency issues
    ROBOT_INITIAL_POSE_FILE = os.path.join(os.path.join(DATA_PATH, "initial_pose.dat"))
    if os.path.exists(ROBOT_INITIAL_POSE_FILE):
        with open(ROBOT_INITIAL_POSE_FILE, "r") as f:
            file = f.read()
            data = file.split(" ")

            # remove few elements from the list
            x, y, z = float(data[3]), float(data[4]), 0.0
            qx, qy, qz, qw = (
                float(data[6]),
                float(data[7]),
                float(data[8]),
                float(data[9]),
            )

            if with_quaternion:
                return x, y, z, qx, qy, qz, qw

            _, _, yaw = quaternion_to_euler((qx, qy, qz, qw))

            return x, y, z, yaw
    else:
        if with_quaternion:
            return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0

        return 0.0, 0.0, 0.0, 0.0
