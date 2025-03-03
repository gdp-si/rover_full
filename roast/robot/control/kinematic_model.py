"""Kinematics model of the robot - Skid steering mechanism"""
import math


class Robot:
    """Skid steering kinematics"""

    WHEEL_RADIUS: float
    ROBOT_LENGTH: float
    ROBOT_WIDTH: float

    front_left = 0.0
    front_right = 0.0
    back_left = 0.0
    back_right = 0.0

    vx = 0.0
    vy = 0.0
    w = 0.0

    x = 0.0
    y = 0.0
    phi = 0.0

    def __init__(
        self, wheel_radius, robot_length, robot_width, max_velocity: float = 2.5
    ) -> None:
        self.WHEEL_RADIUS = wheel_radius
        self.ROBOT_LENGTH = robot_length
        self.ROBOT_WIDTH = robot_width
        self.MAX_VELOCITY = max_velocity

    def calculate_twist(
        self, front_left: float, front_right: float, back_left: float, back_right: float
    ):
        """Calculate the Twist value of the robot

        Args:
            front_left (float): Rotation rate of Wheel 1
            front_right (float): Rotation rate of Wheel 2
            back_left (float): Rotation rate of Wheel 3
            back_right (float): Rotation rate of Wheel 4

        Returns:
            tuple(float, float, float): Linear velocity X, Linear velocity Y, Rotation rate w
        """
        # print (front_left, front_right, back_left, back_right)

        self.vx = (self.WHEEL_RADIUS / 4) * (
            front_left + front_right + back_left + back_right
        )
        self.vy = (self.WHEEL_RADIUS / 4) * (
            front_right - front_left + back_left - back_right
        )
        self.w = (self.WHEEL_RADIUS / (4 * (self.ROBOT_LENGTH + self.ROBOT_WIDTH))) * (
            -front_left + front_right - back_left + back_right
        )

        return self.vx, self.vy, self.w

    def calculate_position(self, vx: float, vy: float, phi: float, dt: float):
        """Calculate the position of the robot

        Args:
            vx (float): Linear velocity in x-direction
            vy (float): Linear velocity in y-direction
            phi (float): Rotation in radians
            dt (float): Time interval in seconds

        Returns:
            tuple(float, float, float): X-coordinate, Y-coordinate, Rotation angle
        """
        # BUG: self.x is not correct
        self.x += vx * dt + self.ROBOT_LENGTH / 2 * math.cos(phi)
        self.y += vy * dt + self.ROBOT_LENGTH / 2 * math.sin(phi)
        self.phi += phi + phi * dt

        return self.x, self.y, self.phi

    def calculate_pose(self, left_distance: float, right_distance: float, w: float):
        """Calculate pose based on the wheel distance and rotation rate

        Args:
            left_distance (float): Distance travelled by left wheel
            right_distance (float): Distance travelled by right wheel
            w (float): Rotation rate in radians per second

        Returns:
            tuple(float, float, float): Linear velocity X, Linear velocity Y, Rotation rate w
        """

        # Map the w within pi range
        if w > 2 * math.pi:
            w = 2 * math.pi
        elif w < -2 * math.pi:
            w = -2 * math.pi

        # Rounding orientation to avoid theta noise
        w = round(w, 2)

        self.x += (
            (2 * self.WHEEL_RADIUS) * (left_distance + right_distance) * math.cos(w)
        )
        self.y += (
            (2 * self.WHEEL_RADIUS) * (left_distance + right_distance) * math.sin(w)
        )
        if not left_distance == 0.0 and not right_distance == 0.0:
            self.phi += (
                (2 * self.WHEEL_RADIUS / (self.ROBOT_LENGTH + self.ROBOT_WIDTH))
                * (right_distance - left_distance)
                * w
            )
            # self.phi += w  # Updated for IMU

            # Map within pi range
            if self.phi > 2 * math.pi:
                self.phi = 2 * math.pi
            elif self.phi < -2 * math.pi:
                self.phi = -2 * math.pi

        return self.x, self.y, self.phi

    @classmethod
    def calculate_speed(cls, vx: float, vy: float):
        """Calculate the speed of the robot using linear velocity components

        Args:
            vx (float): Linear Velocity X
            vy (float): Linear Velocity Y

        Returns:
            float : Speed of the robot in m/s
        """
        speed = math.sqrt(vx**2 + vy**2)
        if speed is math.nan:
            speed = 0
        return speed

    def calculate_wheel_rates(self, vx: float, vy: float, rotation_rate: float):
        """Calculate wheel rotation rate of each wheel

        Args:
            vx (float): Linear velocity in x-direction
            vy (float): Linear velocity in y-direction
            rotation_rate (float): Rotation rate in degrees per second

        Returns:
            tuple(float, float, float, float): Rotation rate of respective wheels
        """

        if vx == 0.0 and vy == 0.0 and rotation_rate == 0.0:
            self.front_left = 0.0
            self.front_right = 0.0
            self.back_left = 0.0
            self.back_right = 0.0
        elif math.nan in (vx, vy, rotation_rate):
            # Teleop control or Emergency stop condition
            self.front_left = 0.0
            self.front_right = 0.0
            self.back_left = 0.0
            self.back_right = 0.0
        else:
            front_left = (1 / self.WHEEL_RADIUS) * (
                vx - vy - (self.ROBOT_LENGTH + self.ROBOT_WIDTH) * rotation_rate
            )
            front_right = (1 / self.WHEEL_RADIUS) * (
                vy + vx + (self.ROBOT_LENGTH + self.ROBOT_WIDTH) * rotation_rate
            )
            back_left = (1 / self.WHEEL_RADIUS) * (
                vy + vx - (self.ROBOT_LENGTH + self.ROBOT_WIDTH) * rotation_rate
            )
            back_right = (1 / self.WHEEL_RADIUS) * (
                vx - vy + (self.ROBOT_LENGTH + self.ROBOT_WIDTH) * rotation_rate
            )

            # TODO: Rewrite ugly patch for robot control

            self.front_left = front_left / 6
            self.front_right = front_right / 6
            self.back_left = back_left / 6
            self.back_right = back_right / 6

        return self.front_left, self.front_right, self.back_left, self.back_right
