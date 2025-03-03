"""Mathematic functions"""
import math

import numpy as np


def rotational_rate_to_rpm(rotational_rate: float):
    """Convert rotational rate to RPM

    Args:
        rotational_rate (float): Rotational rate in rad/s

    Returns:
        float: Rotational rate in RPM
    """
    return rotational_rate * 60 / (2 * math.pi)


def rpm_to_rotational_rate(rpm: int):
    """Convert rpm to rotational rate in rad/s

    Args:
        rpm (int): RPM value

    Returns:
        rotational_rate (float): rotational_rate in rad/s
    """
    return rpm * (2 * math.pi) / 60


def quaternion_to_euler(quaternion: tuple):
    """Convert quaternion to euler angles

    Args:
        quaternion (tuple): Quaternion (x, y, z, w)

    Returns:
        tuple: Euler angles (roll, pitch, yaw)
    """
    x, y, z, w = quaternion
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - z * x))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def euler_to_quaternion(euler: tuple):
    """Convert euler angles to quaternion

    Args:
        euler (tuple): Euler angles (roll, pitch, yaw)

    Returns:
        tuple: Quaternion (x, y, z, w)
    """
    roll, pitch, yaw = euler
    qx = math.sin(0 / 2) * math.cos(0 / 2) * math.cos(yaw / 2) - math.cos(
        0 / 2
    ) * math.sin(0 / 2) * math.sin(yaw / 2)
    qy = math.cos(0 / 2) * math.sin(0 / 2) * math.cos(yaw / 2) + math.sin(
        0 / 2
    ) * math.cos(0 / 2) * math.sin(yaw / 2)
    qz = math.cos(0 / 2) * math.cos(0 / 2) * math.sin(yaw / 2) - math.sin(
        0 / 2
    ) * math.sin(0 / 2) * math.cos(yaw / 2)
    qw = math.cos(0 / 2) * math.cos(0 / 2) * math.cos(yaw / 2) + math.sin(
        0 / 2
    ) * math.sin(0 / 2) * math.sin(yaw / 2)
    return qx, qy, qz, qw


def map_range(value, in_min, in_max, out_min, out_max):
    """Map a value from one range to another range."""
    return out_min + (((value - in_min) / (in_max - in_min)) * (out_max - out_min))


def normalize_angle_positive(angle):
    """
    Wrap the angle between 0 and 2 * pi.
    Args:
        angle (float): angle to wrap.
    Returns:
         The wrapped angle.
    """
    pi_2 = 2.0 * np.pi

    return math.fmod(math.fmod(angle, pi_2) + pi_2, pi_2)


def wrap_angle(angle):
    """
    Wrap the angle between -pi and pi.
    Args:
        angle (float): angle to wrap.
    Returns:
         The wrapped angle.
    """
    a = normalize_angle_positive(angle)
    if a > np.pi:
        a -= 2.0 * np.pi

    return a


def approx(data_1, data_2, tolerence=0.01):
    return np.isclose(data_1, data_2, atol=tolerence)


def euclidean_distance(point_1: tuple, point_2: tuple):
    return np.linalg.norm(np.asarray(point_1) - np.asarray(point_2))
