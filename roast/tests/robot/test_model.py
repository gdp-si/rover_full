"""Test module for kinematics of skid steering robot
"""
import math

import pytest

from roast.robot.control.kinematic_model import Robot


class TestRobot:
    """Test Class for skid steering robot model"""

    R = Robot(wheel_radius=1, robot_length=1, robot_width=1)

    def test_kinematics_init(self):
        """Test initialization of skid steering robot model"""
        assert self.R.WHEEL_RADIUS == 1
        assert self.R.ROBOT_LENGTH == 1
        assert self.R.ROBOT_WIDTH == 1

    @pytest.mark.parametrize("vx, vy", [(1.3, 1.2), (math.nan, math.nan)])
    def test_model_speed(self, vx: float, vy: float):
        """Test speed of the robot model

        Args:
            vx (float): Linear velocity in x direction
            vy (float): Linear velocity in y direction
        """
        pytest.approx(self.R.calculate_speed(vx, vy), math.sqrt(vx**2 + vy**2))

    @pytest.mark.parametrize(
        "r1, r2, r3, r4", [(1, 2, 3, 4), (math.nan, math.nan, math.nan, math.nan)]
    )
    def test_model_twist(self, r1: float, r2: float, r3: float, r4: float):
        """Test twist of the robot model

        Args:
            r1 (float): Rotation rate of Wheel 1
            r2 (float): Rotation rate of Wheel 2
            r3 (float): Rotation rate of Wheel 3
            r4 (float): Rotation rate of Wheel 4
        """
        v_x = self.R.WHEEL_RADIUS / 4 * (r1 - r2 - r3 + r4)
        v_y = self.R.WHEEL_RADIUS / 4 * (r1 + r2 + r3 + r4)
        phi = (
            self.R.WHEEL_RADIUS
            / (4 * (self.R.ROBOT_LENGTH + self.R.ROBOT_WIDTH))
            * (-r1 + r2 - r3 + r4)
        )
        pytest.approx(self.R.calculate_twist(r1, r2, r3, r4), (v_x, v_y, phi))

    @pytest.mark.parametrize(
        "v_x, v_y, phi, dt", [(1, 2, 3, 4), (math.nan, math.nan, math.nan, math.nan)]
    )
    def test_model_position(self, v_x: float, v_y: float, phi: float, dt: float):
        """Test position of the robot model

        Args:
            v_x (float): Linear velocity in x direction
            v_y (float): Linear velocity in y direction
            phi (float): Rotation rate in degrees per second
            dt (float): Time interval in seconds
        """
        x = v_x * dt + self.R.ROBOT_LENGTH / 2 * math.cos(math.radians(phi))
        y = v_y * dt + self.R.ROBOT_LENGTH / 2 * math.sin(math.radians(phi))
        phi = phi + phi * dt
        pytest.approx(self.R.calculate_position(v_x, v_y, phi, dt), (x, y, phi))

    @pytest.mark.parametrize(
        "v_x, v_y, phi",
        [
            (math.nan, math.nan, math.nan),
            (1, 1, 1),
            (0.5, 0.5, 0),
            (0, 0, 0),
            (1, 0, 0),
            (2, 0, 0),
            (2, 2, 0),
            (2, 2, math.pi / 6),
        ],
    )
    def test_model_wheel_rates(self, v_x: float, v_y: float, phi: float):
        """Test wheel rates of the robot model

        Args:
            v_x (float): Linear velocity in x direction
            v_y (float): Linear velocity in y direction
            phi (float): Rotation rate in degrees per second
        """
        r1, r2, r3, r4 = self.R.calculate_wheel_rates(v_x, v_y, phi)

        # assert -self.R.MAX_VELOCITY <= r1 <= self.R.MAX_VELOCITY
        # assert -self.R.MAX_VELOCITY <= r2 <= self.R.MAX_VELOCITY
        # assert -self.R.MAX_VELOCITY <= r3 <= self.R.MAX_VELOCITY
        # assert -self.R.MAX_VELOCITY <= r4 <= self.R.MAX_VELOCITY
