"""Test Module for IMU."""
import concurrent
import os
import unittest
from concurrent.futures import ThreadPoolExecutor
from typing import List

from roast.tests import SkipIfHardwareNotAvailable

try:
    from roast.io import Imu, MotorControl, Range
    from roast.io.configs import ImuConfig, MotorDriveConfig, RangeConfig, UltraConfig
except Exception as e:
    if os.environ["TEST_HARDWARE"].lower() == "false":
        pass
    else:
        raise e


@SkipIfHardwareNotAvailable()
class TestConfig(unittest.TestCase):
    """Test all configs"""

    def test_config(self):
        """Test all configs"""
        _ = UltraConfig
        _ = MotorDriveConfig
        _ = ImuConfig
        _ = RangeConfig


@SkipIfHardwareNotAvailable()
class TestAllHardware(unittest.TestCase):
    """Test all sensors and hardwares"""

    futures: List[concurrent.futures.Future] = []

    def test_all_setup(self):
        """Test all the setup of sensors and hardwares"""
        with ThreadPoolExecutor() as executor:
            self.futures.append(executor.submit(MotorControl, "front"))
            self.futures.append(executor.submit(MotorControl, "back"))
            # self.futures.append(executor.submit(Battery))
            self.futures.append(executor.submit(Range, "left"))
            self.futures.append(executor.submit(Range, "right"))
            self.futures.append(executor.submit(Range, "front"))
            self.futures.append(executor.submit(Imu))

            for future in concurrent.futures.as_completed(self.futures):
                self.assertIsNone(future.exception())

    def test_is_active(self):
        """Test if all sensors are active"""

        for future in concurrent.futures.as_completed(self.futures):
            self.assertIsNone(future.exception())
            self.assertTrue(future.result().is_active())

    def test_get_info(self):
        """Test if all the sensors are getting the info"""

        for future in concurrent.futures.as_completed(self.futures):
            self.assertIsNone(future.exception())
            self.assertTrue(isinstance(future.result().get_info(), str))
            self.assertTrue(len(future.result().get_info()) > 0)

    def test_print_msg(self):
        """Test if all the sensors are printing the info"""

        for future in concurrent.futures.as_completed(self.futures):
            self.assertIsNone(future.exception())
            self.assertIsNone(future.result().print_msg())

    def test_update(self):
        """Test if all the sensors are updating"""

        for future in concurrent.futures.as_completed(self.futures):
            self.assertIsNone(future.exception())
            assert future.result().update()
