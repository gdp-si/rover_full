"""Roast sensors definitions"""
from roast.io.abstract import SensorDefinition
from roast.io.battery import Battery
from roast.io.configs import *  # noqa: F401, F403
from roast.io.imu import Imu
from roast.io.motor_drive import MotorControl
from roast.io.range import Range
from roast.io.speaker import Speaker
from roast.io.ultrasonic import Ultrasonic
from roast.io.weapon_module import ThreatSprayModule

__all__ = [
    "SensorDefinition",
    "Imu",
    "MotorControl",
    "Ultrasonic",
    "Range",
    "ThreatSprayModule",
    "Speaker",
    "Battery",
]
