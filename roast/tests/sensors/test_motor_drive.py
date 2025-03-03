"""Test module for motor drive."""
import os
import unittest
from concurrent.futures import ThreadPoolExecutor

import pytest

from roast.tests import SkipIfHardwareNotAvailable

try:
    from roast.io import MotorDriveConfig
except Exception as e:
    if os.environ["TEST_HARDWARE"].lower() == "false":
        pass
    else:
        raise e


@SkipIfHardwareNotAvailable()
class TestMotorDriveStack(unittest.TestCase):
    """Test cases for the MotorDrive class"""

    def test_setup(self):
        """Test the setup method"""
        fMOTOR_DRIVE = MotorDrive("front")
        bMOTOR_DRIVE = MotorDrive("back")

        self.assertTrue(fMOTOR_DRIVE.is_active())
        self.assertTrue(bMOTOR_DRIVE.is_active())

    def test_setup_parallel(self):
        """Test the setup method in parallel"""

        with ThreadPoolExecutor() as executor:
            for _ in executor.map(MotorDrive, ["front", "back"]):
                return

        raise Exception("Failed to setup motors in parallel")

    def test_is_active(self):
        """Test the is_active method"""
        fMOTOR_DRIVE = MotorDrive("front")
        bMOTOR_DRIVE = MotorDrive("back")

        for _ in range(10):
            self.assertTrue(fMOTOR_DRIVE.is_active())
            self.assertTrue(bMOTOR_DRIVE.is_active())

    def test_is_active_parallel(self):
        """Test the is_active method in parallel


        This test is designed to fail if the motors are not working in parallel in the hardware.
        Probably when the `is_active` method is called in parallel, the motors will throw errors.
        """
        fMOTOR_DRIVE = MotorDrive("front")
        bMOTOR_DRIVE = MotorDrive("back")

        with ThreadPoolExecutor() as executor:
            futures = [executor.submit(fMOTOR_DRIVE.is_active) for _ in range(10)]
            self.assertTrue(all(f.result() for f in futures))
            futures = [executor.submit(bMOTOR_DRIVE.is_active) for _ in range(10)]
            self.assertTrue(all(f.result() for f in futures))

    def test_get_info(self):
        """Test the get_info method"""
        fMOTOR_DRIVE = MotorDrive("front")
        bMOTOR_DRIVE = MotorDrive("back")

        f_info = fMOTOR_DRIVE.get_info()
        b_info = bMOTOR_DRIVE.get_info()

        self.assertTrue(isinstance(f_info, str))
        self.assertTrue(isinstance(b_info, str))

        self.assertTrue(f_info != b_info)

        self.assertTrue(len(f_info) > 0)
        self.assertTrue(len(b_info) > 0)

    def test_get_info_parallel(self):
        """Test the get_info method in parallel"""

        with ThreadPoolExecutor() as executor:
            for result in executor.map(MotorDrive, ["front", "back"]):
                info = result.get_info()
                self.assertTrue(isinstance(info, str))
                self.assertTrue(len(info) > 0)

        raise Exception("Failed to setup motors in parallel")

    def test_print_msg(self):
        """Test the print_msg method"""
        fMOTOR_DRIVE = MotorDrive("front")
        bMOTOR_DRIVE = MotorDrive("back")

        self.assertIsNone(fMOTOR_DRIVE.print_msg())
        self.assertIsNone(bMOTOR_DRIVE.print_msg())

    def assert_update_return(self, data):
        """Assertion function for the update method"""
        self.assertTrue(isinstance(data, tuple))
        self.assertTrue(len(data) == 2)
        self.assertTrue(isinstance(data[0], tuple))
        self.assertTrue(isinstance(data[1], tuple))
        self.assertTrue(len(data[0]) == 2)
        self.assertTrue(len(data[1]) == 2)
        self.assertTrue(isinstance(data[0][0], float))
        self.assertTrue(isinstance(data[0][1], float))
        self.assertTrue(isinstance(data[1][0], float))
        self.assertTrue(isinstance(data[1][1], float))

    def test_update(self):
        """Test the update method"""
        fMOTOR_DRIVE = MotorDrive("front")
        bMOTOR_DRIVE = MotorDrive("back")

        # Test the motor drive
        self.assertTrue(fMOTOR_DRIVE.update(only_odometry=False) is None)
        self.assertTrue(bMOTOR_DRIVE.update(only_odometry=False) is None)

        # Test the encoder data
        data = fMOTOR_DRIVE.update(only_odometry=True)
        self.assert_update_return(data)
        data = bMOTOR_DRIVE.update(only_odometry=True)
        self.assert_update_return(data)

    def test_update_parallel(self):
        """Test the update method in parallel"""

        with ThreadPoolExecutor() as executor:
            for result in executor.map(MotorDrive, ["front", "back"]):
                self.assertTrue(
                    executor.submit(result.update, only_odometry=False), None
                )
                self.assert_update_return(result.update(only_odometry=True))


@SkipIfHardwareNotAvailable()
@pytest.mark.parametrize("motor_side", ["front", "back"])
class TestMotorDrive:
    """Test the motor drive class"""

    def test_setup(self, motor_side):
        """Test the setup method"""
        M = MotorDrive(motor_side)
        assert M.is_active()

    def test_is_active(self, motor_side):
        """Test the is_active method"""
        M = MotorDrive(motor_side)

        for _ in range(10):
            assert M.is_active()

    def test_get_info(self, motor_side):
        """Test the get_info method"""
        M = MotorDrive(motor_side)

        info = M.get_info()
        assert isinstance(info, str)
        assert len(info) > 0

    def test_print_msg(self, motor_side):
        """Test the print_msg method"""
        M = MotorDrive(motor_side)

        assert M.print_msg() is None

    def test_update_motor_drive(self, motor_side):
        """Test the update method for the motor drive"""
        M = MotorDrive(motor_side)

        assert M.update(only_odometry=False) is None

    def test_update_encoder_data(self, motor_side):
        """Test the update method for the encoder data"""
        M = MotorDrive(motor_side)
        assert M.update(only_odometry=True) is not None

        def assert_update_return(data):
            """Assertion function for the update method"""
            assert isinstance(data, tuple)
            assert len(data) == 2
            assert isinstance(data[0], tuple)
            assert isinstance(data[1], tuple)
            assert len(data[0]) == 2
            assert len(data[1]) == 2
            assert isinstance(data[0][0], float)
            assert isinstance(data[0][1], float)
            assert isinstance(data[1][0], float)
            assert isinstance(data[1][1], float)

        data = M.update(only_odometry=True)
        assert_update_return(data)
