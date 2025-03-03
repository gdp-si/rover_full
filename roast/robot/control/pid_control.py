"""PID Control for skid steering robot model"""
import numpy as np

from roast.robot.control.kinematic_model import Robot


class PIDControl:
    """PID Control for skid steering robot model"""

    def __init__(
        self, robot: Robot, Kp: float, Ki: float, Kd: float, max_speed: float = 1.0
    ):
        """PID Control for skid steering robot model

        Args:
            Kp (float): Kp gain
            Ki (float): Ki gain
            Kd (float): Kd gain
            max_speed (float): Maximum velocity in m/s
        """
        self.robot = robot
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.max_speed = max_speed
        self.integral: np.ndarray = np.zeros(3)
        self.derivative: np.ndarray = np.zeros(3)
        self.last_error: np.ndarray = np.zeros(3)
        self.error: np.ndarray = np.zeros(3)
        self.last_time = 0

    def set_gains(self, Kp, Ki, Kd, max_speed):
        """Set the PID gains"""
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.max_speed = max_speed

    def update(self, error: tuple, dt: float = 0.1):
        """Motor Control for skid steering robot model

        Args:
            error (tuple(float, float, float)): Error in x, y, and theta
            dt (float): time difference in seconds. Default to 0.1

        Returns:
            tuple(float, float, float, float): Front left, front right, back left, back right speed
        """
        if np.isnan(error).any():
            self.error = np.zeros(3)
            self.integral = np.zeros(3)
            self.derivative = np.zeros(3)

        if len(error) < 3:
            raise ValueError("Error must be a tuple of length 3")

        self.error = np.array(error)

        if self.last_time == 0:
            self.integral = np.multiply(self.error, dt)

            # Use only Integral on first run
            control = self.Ki * self.integral

            self.last_error = self.error
        else:
            self.integral += np.multiply(self.error, dt)
            self.derivative += (self.error - self.last_error) / dt

            control = (
                self.Kp * self.error
                + self.Ki * self.integral
                + self.Kd * self.derivative
            )

            self.last_error = self.error

        (
            front_left,
            front_right,
            back_left,
            back_right,
        ) = self.robot.calculate_wheel_rates(control[0], control[1], control[2])

        # Check velocity limits
        if front_left > self.max_speed:
            front_left = self.max_speed
        if front_left < -1 * self.max_speed:
            front_left = -1 * self.max_speed

        if front_right > self.max_speed:
            front_right = self.max_speed
        if front_right < -1 * self.max_speed:
            front_right = -1 * self.max_speed

        if back_left > self.max_speed:
            back_left = self.max_speed
        if back_left < -1 * self.max_speed:
            back_left = -1 * self.max_speed

        if back_right > self.max_speed:
            back_right = self.max_speed
        if back_right < -1 * self.max_speed:
            back_right = -1 * self.max_speed

        return front_left, front_right, back_left, back_right

    def reset(self):
        """Reset the PID controller"""
        self.integral = np.array([0.0, 0.0, 0.0])
        self.derivative = np.array([0.0, 0.0, 0.0])
        self.last_error = np.array([0.0, 0.0, 0.0])
        self.last_time = 0.0
