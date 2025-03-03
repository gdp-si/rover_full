"""Test module for PID controller"""
import numpy as np
import pytest

from roast.robot.control.kinematic_model import Robot
from roast.robot.control.pid_control import PIDControl


@pytest.mark.skip(reason="TODO: Fix this test")
class TestPID:
    """Test Class for PID Control"""

    R = Robot(wheel_radius=1, robot_length=1, robot_width=1)

    def test_pid_control_init(self):
        """Test initialization of PID controller"""
        pid = PIDControl(self.R, Kp=1, Ki=1, Kd=1)
        assert pid.Kp == 1
        assert pid.Ki == 1
        assert pid.Kd == 1
        assert pid.robot == self.R

    def test_pid_control_set_gains(self):
        """Test set gains of PID controller"""
        pid = PIDControl(self.R, Kp=1, Ki=1, Kd=1)
        pid.set_gains(Kp=2, Ki=2, Kd=2, max_speed=1)
        assert pid.Kp == 2
        assert pid.Ki == 2
        assert pid.Kd == 2
        assert pid.max_speed == 1

    def test_pid_control_reset(self):
        """Test PID reset"""
        pid = PIDControl(self.R, Kp=1, Ki=1, Kd=1)
        pid.reset()
        assert pid.integral is not None
        assert pid.last_error is not None
        assert pid.last_time == 0

        pytest.approx(pid.integral, 0)
        pytest.approx(pid.derivative, 0)
        pytest.approx(pid.last_error, 0)

    @pytest.mark.parametrize(
        "time, error",
        [
            (0, (0, 0, 0)),
            (1, (1, 1, 1)),
            (2, (2, 2, 2)),
            (2, (np.nan, np.nan, np.nan)),
            (0, (1, 1, 1)),
            (0, (2, 2, 2)),
            (0, (np.nan, np.nan, np.nan)),
        ],
    )
    def test_pid_control_test(self, time, error):
        """Test PID control"""
        pid = PIDControl(self.R, Kp=1, Ki=1, Kd=1)
        fl, fr, bl, br = pid.update(time, error)

        # Check that the speed is within the max speed
        assert fl <= pid.max_speed and fr <= pid.max_speed
        assert bl <= pid.max_speed and br <= pid.max_speed

        # Check that the speed is above min speed
        assert fl >= -1 * pid.max_speed and fr >= -1 * pid.max_speed
        assert bl >= -1 * pid.max_speed and br >= -1 * pid.max_speed

    @pytest.mark.parametrize(
        "time, error",
        [
            (0, (0, 0, 0)),
            (1, (1, 1, 1)),
            (2, (2, 2, 2)),
            (2, (np.nan, np.nan, np.nan)),
            (0, (1, 1, 1)),
            (0, (2, 2, 2)),
            (0, (np.nan, np.nan, np.nan)),
        ],
    )
    def test_pid_control_integral(self, time, error):
        """Test PID integral"""
        pid = PIDControl(self.R, Kp=1, Ki=1, Kd=1)
        pid.reset()
        pid.update(time, error)

        # Check if error is nan
        if np.isnan(error).any():
            pytest.approx(pid.integral, np.zeros(3))
        else:
            pytest.approx(pid.integral, np.multiply(error, time))

    @pytest.mark.parametrize(
        "time, error",
        [(0, (0, 0, 0)), (1, (1, 1, 1)), (2, (2, 2, 2)), (2, (np.nan, np.nan, np.nan))],
    )
    def test_pid_control_pid(self, time, error):
        """Test PID control"""
        pid = PIDControl(self.R, Kp=1, Ki=1, Kd=1)

        pid.last_time = time
        pid.reset()
        pid.update(time, error)

        # PID Term
        if np.isnan(error).any():
            pytest.approx(pid.integral, np.zeros(3))
            pytest.approx(pid.derivative, np.zeros(3))
        else:
            pytest.approx(pid.last_error, error)
            pytest.approx(pid.integral, np.multiply(error, time))

            # Prevent division by zero
            time = time if time != 0 else 0.0001
            error = np.array(error) - pid.last_error
            pytest.approx(pid.derivative, np.divide(error, time))
