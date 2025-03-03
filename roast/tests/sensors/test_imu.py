"""Test Module for IMU."""
import os
import unittest

from roast.tests import SkipIfHardwareNotAvailable

try:
    from roast.io import Imu
except Exception as e:
    if os.environ["TEST_HARDWARE"].lower() == "false":
        pass
    else:
        raise e


@SkipIfHardwareNotAvailable()
class TestIMUSensor(unittest.TestCase):
    """Test cases for the IMU class"""

    def test_setup(self):
        """Setup the test environment"""
        IMU = Imu()
        self.assertTrue(IMU.is_active())

    def test_is_active(self):
        """Test the is_active method"""
        IMU = Imu()

        # Continuous check if the IMU is active
        for _ in range(10):
            self.assertTrue(IMU.is_active())

    def test_get_info(self):
        """Test the get_info method"""
        IMU = Imu()
        info = IMU.get_info()

        self.assertTrue(isinstance(info, str))
        self.assertTrue(len(info) > 0)

    def test_print_msg(self):
        """Test the print_msg method"""
        IMU = Imu()

        self.assertIsNone(IMU.print_msg())

    def test_update(self):
        """Test the update method"""
        IMU = Imu()
        data = IMU.update()

        self.assertTrue(isinstance(data, dict))
        for key, value in data.items():
            self.assertTrue(isinstance(value, float))
            self.assertTrue(
                key in {"accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z"}
            )
