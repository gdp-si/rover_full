"""Test module for battery."""
import os
import unittest

from roast.tests import SkipIfHardwareNotAvailable

try:
    from roast.io import Battery
except Exception as e:
    if os.environ["TEST_HARDWARE"].lower() == "false":
        pass
    else:
        raise e


@SkipIfHardwareNotAvailable()
class TestBattery(unittest.TestCase):
    """Test the Battery class"""

    def test_setup(self):
        """Test the setup method"""
        BAT = Battery()

        self.assertTrue(BAT.is_active())

    def test_is_active(self):
        """Test the is_active method"""
        BAT = Battery()

        for _ in range(10):
            self.assertTrue(BAT.is_active())

    def test_get_info(self):
        """Test the get_info method"""
        BAT = Battery()

        info = BAT.get_info()

        self.assertTrue(isinstance(info, str))
        self.assertTrue(len(info) > 0)

    def test_print_msg(self):
        """Test the print_msg method"""
        BAT = Battery()

        self.assertIsNone(BAT.print_msg())

    def test_update(self):
        """Test the update method"""
        BAT = Battery()
        data = BAT.update()

        self.assertTrue(isinstance(data, dict))
        for key, value in data.items():
            self.assertTrue(isinstance(value, float))
            self.assertTrue(
                key
                in {
                    "battery_percentage",
                    "battery_temperature",
                    "battery_voltage",
                    "battery_current_capacity",
                    "battery_current_consumption",
                }
            )
