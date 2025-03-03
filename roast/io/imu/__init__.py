"""Imu interface for ROAST"""
from roast import RobotProfile

if RobotProfile.ARDUINO_IMU:
    from roast.io.imu.arduino_interface import Imu  # type: ignore
else:
    from roast.io.imu.i2c_interface import Imu  # type: ignore

__all__ = ["Imu"]
