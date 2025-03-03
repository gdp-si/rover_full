"""Test Module for IMU."""
import os
import unittest
from concurrent.futures import ThreadPoolExecutor

import pytest

from roast.tests import SkipIfHardwareNotAvailable

try:
    from roast.io import Range
except Exception as e:
    if os.environ["TEST_HARDWARE"].lower() == "false":
        pass
    else:
        raise e


@SkipIfHardwareNotAvailable()
@pytest.mark.parametrize("sensor", ["left", "right", "front"])
class TestRangeSensor:
    """Test the range sensor"""

    def test_setup(self, sensor):
        """Test the setup"""
        R = Range(sensor)

        assert R.is_active()

    def test_is_active(self, sensor):
        """Test the is_active method"""
        R = Range(sensor)

        for _ in range(10):
            assert R.is_active()

    def test_get_info(self, sensor):
        """Test the get_info method"""
        R = Range(sensor)
        info = R.get_info()

        assert isinstance(info, str)
        assert len(info) > 0

    def test_print_msg(self, sensor):
        """Test the print_msg method"""
        R = Range(sensor)

        assert R.print_msg() is None

    def test_update(self, sensor):
        """Test the update method"""
        R = Range(sensor)
        data = R.update()

        assert isinstance(data, float)
        assert 0 <= data <= 4.0
