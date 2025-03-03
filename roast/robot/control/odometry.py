"""Module to calculate motor odometry for the 4-wheeled skid steering robot."""
import math
import os

from roast.glogging import Logger


class Odometry:
    """Motor Odometry Class definition

    REF: http://www-personal.umich.edu/~johannb/Papers/umbmark.pdf
    """

    LOG = Logger(_module_name=os.path.basename(__file__))

    def __init__(self, wheel_radius: float, wheel_base: float, robot_width: float):
        """Initialize the Odometry class."""
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0  # in radians
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.robot_width = robot_width

        self._prev_left_encoder = 0.0
        self._prev_right_encoder = 0.0

    @property
    def x(self):
        """Return the x position."""
        return self._x

    @x.setter
    def x(self, x):
        """Set the x position."""
        self._x = x

    @property
    def y(self):
        """Return the y position."""
        return self._y

    @y.setter
    def y(self, y):
        """Set the y position."""
        self._y = y

    @property
    def theta(self):
        """Return the theta position."""
        return self._theta

    @theta.setter
    def theta(self, theta):
        """Set the theta position."""
        self._theta = theta

    def update(
        self,
        left_encoder: float,
        right_encoder: float,
        yaw: float,
        dt: float = 0.01,  # pylint: disable=unused-argument
    ):
        """Get odometry coordinates from left and right distance travelled

         - Model - Virtual Differential Model

        Args:
            left_encoder (float): relative distance travelled by left encoder
            right_encoder (float): relative distance travelled by right encoder
            yaw (float): relative yaw angle in radians
            dt (float, optional): relative time taken. Defaults to 0.01.

        """
        d_right_wheel = right_encoder - self._prev_right_encoder
        d_left_wheel = left_encoder - self._prev_left_encoder

        # Robot distance travelled
        d = (d_right_wheel + d_left_wheel) / 2

        # self._theta += (d_right_wheel - d_left_wheel) / self.wheel_base
        self._theta = yaw
        self._x = self._x + math.cos(self._theta) * d
        self._y = self._y + math.sin(self._theta) * d

        self._prev_right_encoder = right_encoder
        self._prev_left_encoder = left_encoder

        return self._x, self._y, self._theta

    def reset(self):
        """Reset the odometry values."""
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
