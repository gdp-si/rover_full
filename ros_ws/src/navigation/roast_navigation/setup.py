"""Setup file for roast_navigation."""
import os
from glob import glob

from setuptools import setup

package_name = "roast_navigation"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name, f"{package_name}.libs"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch/"), glob("./launch/*")),
        (os.path.join("share", package_name, "config/"), glob("./config/*")),
        (
            os.path.join("share", package_name, "maps/"),
            list(glob("./maps/**/*.pgm", recursive=True)),
        ),
        (
            os.path.join("share", package_name, "maps/"),
            list(glob("./maps/**/*.yaml", recursive=True)),
        ),
        (
            os.path.join("share", package_name, "behavior_trees/"),
            glob("./behavior_trees/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Selvakumar",
    maintainer_email="franklinselva10@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "initial_pose_handler = roast_navigation.initial_pose_handler:main",
            "odom_pose_handler = roast_navigation.odometry_pose_handler:main",
        ],
    },
)
