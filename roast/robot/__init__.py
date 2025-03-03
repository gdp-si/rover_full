"""Roast Robot Related API.

This module contains information about the Roast Robot.

Available Resources:
- RobotParameters: Hardware and Geometric Information about the robot.
"""
from roast.profiles import RobotProfile
from roast.robot.control import Robot
from roast.utils import compare_tuple

if RobotProfile.name == "HOST":
    from roast.robot.configs import HostRobotParameters as RobotParameters  # type: ignore
elif RobotProfile.name == "BEAST":
    from roast.robot.configs import BeastRobotParameters as RobotParameters  # type: ignore
elif RobotProfile.name == "SIM":
    from roast.robot.configs import SimRobotParameters as RobotParameters  # type: ignore
elif RobotProfile.name == "SAURUS":
    from roast.robot.configs import SaurusRobotParameters as RobotParameters  # type: ignore
elif RobotProfile.name == "LEUKO":
    from roast.robot.configs import LeukoRobotParameters as RobotParameters  # type: ignore
elif RobotProfile.name == "MINI_ROBOT":
    from roast.robot.configs import mini_robotRobotParameters as RobotParameters  # type: ignore
elif RobotProfile.name == "LEUKO":
    from roast.robot.configs import LeukoRobotParameters as RobotParameters  # type: ignore
elif RobotProfile.name == "AJAI_CHICAGO_B1":
    from roast.robot.configs import AjaiChicagoB1RobotParameters as RobotParameters  # type: ignore

assert compare_tuple(
    RobotProfile.OAKD_ARRAY_LIST, tuple(RobotParameters.SENSOR_POSITIONS.keys())
), f"OAKD Sensor Positions are not well-defined for this robot profile: {RobotProfile.name}"
assert compare_tuple(
    RobotProfile.TOF_SENSOR_ARRAY_LIST, tuple(RobotParameters.SENSOR_POSITIONS.keys())
), f"TOF Sensor Positions are not well-defined for this robot profile: {RobotProfile.name}"
assert compare_tuple(
    RobotProfile.ULTRASONIC_SENSOR_ARRAY_LIST,
    tuple(RobotParameters.SENSOR_POSITIONS.keys()),
), f"Ultrasonic Sensor Positions are not well-defined for this robot profile: {RobotProfile.name}"

__all__ = ["Robot", "RobotParameters"]
