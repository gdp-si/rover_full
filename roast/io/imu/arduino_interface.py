"""IMU sensor class"""
import time

import serial

from roast.glogging import Logger
from roast.io import PortParser
from roast.utils.math import euler_to_quaternion
from roast.utils.patterns import ThreadSafeSingletonMetaclass

PORTS = PortParser()()


class Imu(metaclass=ThreadSafeSingletonMetaclass):
    LOG = Logger("IMU")

    def __init__(self):
        self._serial = serial.Serial(PORTS["imu"][1], 115200)
        time.sleep(1)  # Adjust the delay as needed

    def calibrate_imu(self):
        return True

    def is_active(self):
        return True

    def update(self):
        self._serial.reset_input_buffer()
        while True:
            line = self._serial.readline().decode("utf-8", errors="ignore").strip()
            if line.startswith("Orientation:"):
                try:
                    data = line.split("Orientation:")[1].strip().split()
                    yaw = float(data[0])
                    pitch = float(data[1])
                    roll = float(data[2])

                    yaw = 360 - yaw

                    if yaw > 180:
                        yaw = yaw - 360
                    else:
                        yaw = yaw

                    pitch = pitch * 3.14159 / 180
                    roll = roll * 3.14159 / 180
                    yaw = yaw * 3.14159 / 180
                    (x, y, z, w) = euler_to_quaternion((roll, pitch, yaw))
                    return {
                        "quaternion": [x, y, z, w],
                        "linear_acceleration": [0.0, 0.0, 0.0],
                    }
                except:
                    pass

    def destroy(self):
        self._serial.close()
