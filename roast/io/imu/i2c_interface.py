"""IMU sensor class"""
import Adafruit_GPIO.I2C as I2C

from roast.glogging import Logger
from roast.io.libs.bno055 import BNO055
from roast.utils.math import approx, euler_to_quaternion
from roast.utils.patterns import ThreadSafeSingletonMetaclass


class Imu(metaclass=ThreadSafeSingletonMetaclass):
    LOG = Logger("IMU")

    def __init__(self):
        i2c_device = I2C.get_i2c_device(0x28, busnum=7)
        self.imu = BNO055(i2c_device=i2c_device)
        self.imu.begin()

    def calibrate_imu(self):
        """Retains the previous known IMU data fom sensor registers"""

        self.LOG.INFO("calibrating...")
        while not bool(self.imu.get_calibration_status() == (3, 3, 3, 3)):
            data = self.imu.get_calibration()
            self.imu.set_calibration(data)
        return True

    def is_active(self):
        return bool(approx(self.imu.read_gravity()[2], 9.8, tolerence=0.1))

    def update(self):
        ax, ay, az = self.imu.read_linear_acceleration()
        # euler is selected as we restrict the orientation to 180 degrees
        yaw, roll, pitch = self.imu.read_euler()

        yaw = 360 - yaw

        if yaw > 180:
            yaw = yaw - 360
        else:
            yaw = yaw

        pitch = pitch * 3.14159 / 180
        roll = roll * 3.14159 / 180
        yaw = yaw * 3.14159 / 180

        # Wrap the angle to +/-180
        # yaw = wrap_angle(yaw)
        (x, y, z, w) = euler_to_quaternion((roll, pitch, yaw))

        return {"quaternion": [x, y, z, w], "linear_acceleration": [ax, ay, az]}

    def destroy(self):
        return True
