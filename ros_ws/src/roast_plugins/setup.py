import os
from glob import glob

from setuptools import setup

package_name = "roast_plugins"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Load files into share directory
        (os.path.join("share", package_name, "launch/"), glob("./launch/*")),
        (os.path.join("share", package_name, "configs/"), glob("./configs/*")),
        (os.path.join("share", package_name, "maps/"), glob("./maps/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Selvakumar.H-S",
    maintainer_email="franklinselva10@gmail.com",
    description="Roast ROS plugins for sensors and motors",
    license="BSD3",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Hardware plugins
            "imu = roast_plugins.hardware_nodes.imu:main",
            "odometry = roast_plugins.hardware_nodes.odometry:main",
            "ultrasonic = roast_plugins.hardware_nodes.ultrasonic:main",
            "oakd_array = roast_plugins.hardware_nodes.oakd_array:main",
            "robot_control = roast_plugins.hardware_nodes.robot_control:main",
            "range = roast_plugins.hardware_nodes.range:main",
            "threat_spray_module = roast_plugins.hardware_nodes.threat_spray_module:main",
            "speaker = roast_plugins.hardware_nodes.speaker:main",
            "display_node = roast_plugins.hardware_nodes.display:main",
            # Other plugins
            "robot_state_publisher = roast_plugins.robot_state_publisher:main",
            "laser_time_sync = roast_plugins.laser_time_sync:main",
        ],
    },
)
