import os
from glob import glob

from setuptools import setup

package_name = "roast_utils"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "behavior_trees/"),
            glob("./behavior_trees/*"),
        ),
        (
            os.path.join("share", package_name, "config/"),
            glob("./config/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Selvakumar.H-S",
    maintainer_email="franklinselva10@gmail.com",
    description="A collection of ROS2 utility nodes for roast project",
    license="GPT-3",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "acceleration_estimator = roast_utils.acceleration_estimator:main",
            "motor_drive_calibrator = roast_utils.motor_drive_calibrator:main",
            "pid_tuner = roast_utils.pid_tuner:main",
            "odometry_tester = roast_utils.odometry_tester:main",
            "patrol_points_builder = roast_utils.patrol_points_builder:main",
            "threat_pose_publisher = roast_utils.threat_pose_publisher:main",
            "threat_tracking_with_patrol = roast_utils.threat_tracking_with_patrol:main",
            "rosbag2csv = roast_utils.rosbag2csv:main",
            "lifecycle_activator = roast_utils.lifecycle_activator:main",
        ],
    },
)
