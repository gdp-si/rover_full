"""Test module for motor odometry."""
import numpy as np
import pytest

from roast.robot import RobotParameters
from roast.robot.control import Odometry
from roast.tests import SkipIfHardwareNotAvailable


## function to generate odometry distance
def wheel_distance(_from=0.0, _to=0.0, _increment=0.007):
    """Function to generate wheel odometry distance as an array based on incremental distance."""
    return list(np.arange(_from, _to, _increment))


wheel_radius = RobotParameters.wheel_diameter / 2
wheel_base = RobotParameters.wheel_base
robot_width = RobotParameters.robot_width


class TestOdometry:
    """Test the odometry class."""

    @staticmethod
    def test_setup():
        """Test the setup of the odometry class."""
        odom = Odometry(
            wheel_base=wheel_base, wheel_radius=wheel_radius, robot_width=robot_width
        )

        assert odom.x == 0.0
        assert odom.y == 0.0
        assert odom.theta == 0.0

    @staticmethod
    def test_reset():
        """Test the reset of the odometry class."""
        odom = Odometry(
            wheel_base=wheel_base, wheel_radius=wheel_radius, robot_width=robot_width
        )
        odom.x = 1.0
        odom.y = 2.0
        odom.theta = 3.0

        odom.reset()

        assert odom.x == 0.0
        assert odom.y == 0.0
        assert odom.theta == 0.0

    @staticmethod
    @pytest.mark.parametrize(
        "distance", [0.15, 0.2, 0.3, 0.4, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0]
    )
    def test_odometry_straight_path(distance):
        """Test odometry for straight path for 1 metre"""
        odom = Odometry(
            wheel_base=wheel_base, wheel_radius=wheel_radius, robot_width=robot_width
        )

        distance_array = wheel_distance(_to=distance)
        for left_distance, right_distance in zip(distance_array, distance_array):
            odom.update(left_distance, right_distance, 0.0)

        assert pytest.approx(odom.x, 0.05) == distance
        assert pytest.approx(odom.y, 0.05) == 0.0
        assert pytest.approx(odom.theta, 0.05) == 0.0

        distance_array.reverse()
        for left_distance, right_distance in zip(distance_array, distance_array):
            odom.update(left_distance, right_distance, 0.0)

        assert pytest.approx(odom.x, 0.05) == 0.0
        assert pytest.approx(odom.y, 0.05) == 0.0
        assert pytest.approx(odom.theta, 0.05) == 0.0

    @staticmethod
    @pytest.mark.parametrize(
        "distance", [0.15, 0.2, 0.3, 0.4, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0]
    )
    def test_odometry_reverse_straight_path(distance):
        """Test odometry for straight path for 1 metre"""
        odom = Odometry(
            wheel_base=wheel_base, wheel_radius=wheel_radius, robot_width=robot_width
        )

        distance_array = wheel_distance(_to=distance)

        for left_distance, right_distance in zip(distance_array, distance_array):
            odom.update(-left_distance, -right_distance, 0.0)

        assert pytest.approx(odom.x, 0.05) == -distance
        assert pytest.approx(odom.y, 0.05) == 0.0

        distance_array.reverse()
        for left_distance, right_distance in zip(distance_array, distance_array):
            odom.update(-left_distance, -right_distance, 0.0)

        assert pytest.approx(odom.x, 0.05) == 0.0
        assert pytest.approx(odom.y, 0.05) == 0.0
        assert pytest.approx(odom.theta, 0.05) == 0.0

    @staticmethod
    @SkipIfHardwareNotAvailable()
    @pytest.mark.parametrize(
        "distance", [0.15, 0.2, 0.3, 0.4, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0]
    )
    def test_odometry_right_turn(distance):
        """Test odometry of the robot given the size of the square path"""
        odom = Odometry(
            wheel_base=wheel_base, wheel_radius=wheel_radius, robot_width=robot_width
        )

        distance_array = wheel_distance(_to=distance)

        for left_distance, right_distance in zip(distance_array, distance_array):
            odom.update(left_distance, -right_distance, 0.0)

        assert pytest.approx(odom.x, 0.05) == 0.0
        assert pytest.approx(odom.y, 0.05) == 0.0

        distance_array.reverse()
        for left_distance, right_distance in zip(distance_array, distance_array):
            odom.update(-left_distance, right_distance, 0.0)

        assert pytest.approx(odom.x, 0.05) == 0.0
        assert pytest.approx(odom.y, 0.05) == 0.0

    @staticmethod
    @SkipIfHardwareNotAvailable()
    @pytest.mark.parametrize(
        "distance", [0.15, 0.2, 0.3, 0.4, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0]
    )
    def test_odometry_left_turn(distance):
        """Test odometry of the robot in left turn"""
        odom = Odometry(
            wheel_base=wheel_base, wheel_radius=wheel_radius, robot_width=robot_width
        )

        distance_array = wheel_distance(_to=distance)

        for left_distance, right_distance in zip(distance_array, distance_array):
            odom.update(-left_distance, right_distance, 0.0)

        assert pytest.approx(odom.x, 0.05) == 0.0
        assert pytest.approx(odom.y, 0.05) == 0.0

        distance_array.reverse()
        for left_distance, right_distance in zip(distance_array, distance_array):
            odom.update(left_distance, -right_distance, 0.0)

        assert pytest.approx(odom.x, 0.05) == 0.0
        assert pytest.approx(odom.y, 0.05) == 0.0

    @staticmethod
    @SkipIfHardwareNotAvailable()
    @pytest.mark.parametrize(
        "distance", [0.15, 0.2, 0.3, 0.4, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0]
    )
    def test_odometry_l_path(distance):
        """Test odometry of the robot in L path"""
        odom = Odometry(
            wheel_base=wheel_base, wheel_radius=wheel_radius, robot_width=robot_width
        )

        distance_array = wheel_distance(_from=0.0, _to=distance)

        for left_distance, right_distance in zip(distance_array, distance_array):
            odom.update(left_distance, right_distance, 0.0)

        distance_array = wheel_distance(_from=distance, _to=2 * distance)

        for left_distance, right_distance in zip(distance_array, distance_array):
            odom.update(left_distance, right_distance, 0.0)

        assert pytest.approx(odom.x, 0.05) == distance
        assert pytest.approx(odom.y, 0.05) == distance

    @staticmethod
    @SkipIfHardwareNotAvailable()
    @pytest.mark.parametrize(
        "distance", [0.15, 0.2, 0.3, 0.4, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0]
    )
    def test_odometry_inverted_l_path(distance):
        """Test odometry of the robot in L path"""
        odom = Odometry(
            wheel_base=wheel_base, wheel_radius=wheel_radius, robot_width=robot_width
        )

        distance_array = wheel_distance(_from=0.0, _to=distance)

        for left_distance, right_distance in zip(distance_array, distance_array):
            odom.update(left_distance, right_distance, 0.0)

        assert pytest.approx(odom.x, 0.05) == distance
        assert pytest.approx(odom.y, 0.05) == 0.0

        distance_array = wheel_distance(_from=distance, _to=2 * distance)

        for left_distance, right_distance in zip(distance_array, distance_array):
            odom.update(left_distance, right_distance, 0.0)

        assert pytest.approx(odom.x, 0.05) == distance
        assert pytest.approx(odom.y, 0.05) == -distance

    # FIXME: Dumb way to test this. Need to update a better way to test this.
    @staticmethod
    @SkipIfHardwareNotAvailable()
    @pytest.mark.parametrize("distance", [1.0, 2.0, 3.0, 4.0, 5.0])
    def test_odometry_square_path(distance):
        """Test odometry of the robot given the size of the square path"""
        odom = Odometry(
            wheel_base=wheel_base, wheel_radius=wheel_radius, robot_width=robot_width
        )

        distance_array = wheel_distance(_from=0.0, _to=distance)

        # First side
        for left_distance, right_distance in zip(distance_array, distance_array):
            odom.update(left_distance, right_distance, 0.0)

        assert pytest.approx(odom.x, 0.05) == distance
        assert pytest.approx(odom.y, 0.05) == 0.0

        # Second side
        distance_array = wheel_distance(_from=distance, _to=2 * distance)

        for left_distance, right_distance in zip(distance_array, distance_array):
            odom.update(left_distance, right_distance, 0.0)

        assert pytest.approx(odom.x, 0.05) == distance
        assert pytest.approx(odom.y, 0.05) == distance

        # Third side
        distance_array = wheel_distance(_from=2 * distance, _to=3 * distance)
        for left_distance, right_distance in zip(distance_array, distance_array):
            odom.update(left_distance, right_distance, 0.0)

        assert pytest.approx(round(odom.x, 1), 0.05) == 0.0
        assert pytest.approx(odom.y, 0.05) == distance

        # Fourth side
        distance_array = wheel_distance(_from=3 * distance, _to=4 * distance)
        for left_distance, right_distance in zip(distance_array, distance_array):
            odom.update(left_distance, right_distance, 0.0)

        assert pytest.approx(round(odom.x, 1), 0.05) == 0.0
        assert pytest.approx(round(odom.y, 1), 0.05) == 0.0

    @staticmethod
    @SkipIfHardwareNotAvailable()
    @pytest.mark.parametrize("distance", [1.0, 2.0, 3.0, 4.0, 5.0])
    def test_odometry_inverted_square_path(distance):
        """Test odometry of the robot given the size of the square path"""
        odom = Odometry(
            wheel_base=wheel_base, wheel_radius=wheel_radius, robot_width=robot_width
        )

        distance_array = wheel_distance(_from=0.0, _to=distance)

        # First side
        for left_distance, right_distance in zip(distance_array, distance_array):
            odom.update(left_distance, right_distance, 0.0)

        assert pytest.approx(odom.x, 0.05) == distance
        assert pytest.approx(odom.y, 0.05) == 0.0

        # Second side
        distance_array = wheel_distance(_from=distance, _to=2 * distance)

        for left_distance, right_distance in zip(distance_array, distance_array):
            odom.update(left_distance, right_distance, 0.0)

        assert pytest.approx(odom.x, 0.05) == distance
        assert pytest.approx(odom.y, 0.05) == -distance

        # Third side
        distance_array = wheel_distance(_from=2 * distance, _to=3 * distance)
        for left_distance, right_distance in zip(distance_array, distance_array):
            odom.update(left_distance, right_distance, 0.0)

        assert pytest.approx(round(odom.x, 1), 0.05) == 0.0
        assert pytest.approx(odom.y, 0.05) == -distance

        # Fourth side
        distance_array = wheel_distance(_from=3 * distance, _to=4 * distance)
        for left_distance, right_distance in zip(distance_array, distance_array):
            odom.update(left_distance, right_distance, 0.0)

        assert pytest.approx(round(odom.x, 1), 0.05) == 0.0
        assert pytest.approx(round(odom.y, 1), 0.05) == 0.0
