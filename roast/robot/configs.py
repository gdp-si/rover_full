"""Robot related configurations and Parameters

All parameters follow SI units
"""
from typing import List


# pylint: disable=too-few-public-methods
class FourInchRobotParameters:
    """Robot parameters like base plate are, wheel diameter, etc."""

    wheel_diameter = 0.101
    robot_length = 0.4
    robot_width = 0.430
    robot_height = 0.700
    wheel_base = 0.343  # Distance between wheel to wheel along x-axis
    track_width = 0.380  # Distance between wheel to wheel along y-axis

    min_rotation_rate = 1.0
    max_rotation_rate = 5.3  # wheel speed limit
    robot_velocity_limit = (
        4.3  # robot speed limit (patch: scaled down by a factor of 8)
    )

    threshold_range = 0.2  # Threshold for range sensor

    # Control Parameters for PID
    Kp = 1.0
    Kd = 0.1
    Ki = 0.1

    # Encoders parameters
    encoders_window_size = 3

    # The below parameters are for the modules and wheels with respect to the base frame
    # Base frame - Centre of the robot base plate.
    PUBLISH_RATE = {
        "imu": 50,
        "status": 0.2,
        "camera": 30,
        "lidar": 10,
        "ultrasonic": 10,
        "encoder": 30,
        "odometry": 30,  # Odometry rate would be the same as TF rate
    }

    ROBOT_BASE_FRAME = {"x": 0.40, "y": 0.40, "z": 0.2}

    # Sensor Positions
    SENSOR_POSITIONS = {
        "imu_link": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.060,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "velodyne": {
            "x": 0.065,
            "y": 0.0,
            "z": 0.670,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "tof_front_left": {
            "x": 0.298,
            "y": 0.085,
            "z": 0.145,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.5235,
        },
        "tof_front_right": {
            "x": 0.298,
            "y": -0.085,
            "z": 0.154,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": -0.5235,
        },
        "tof_front_middle": {
            "x": 0.325,
            "y": 0.0,
            "z": 0.145,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "tof_right_middle": {
            "x": 0.179,
            "y": -0.173,
            "z": 0.145,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": -1.5708,
        },
        "tof_right_front": {
            "x": 0.250,
            "y": -0.150,
            "z": 0.145,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": -1.0472,
        },
        "tof_left_front": {
            "x": 0.250,
            "y": 0.150,
            "z": 0.145,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 1.0472,
        },
        "tof_left_middle": {
            "x": 0.179,
            "y": 0.173,
            "z": 0.145,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 1.5708,
        },
        "front_oakd_camera": {  # oak-d base frame
            "x": 0.065,
            "y": 0.0,
            "z": 0.867,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "left_oakd_camera": {
            "x": 0.0,
            "y": 0.065,
            "z": 0.867,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 1.57,
        },
        "right_oakd_camera": {
            "x": 0.0,
            "y": -0.065,
            "z": 0.867,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": -1.57,
        },
        "back_oakd_camera": {
            "x": -0.065,
            "y": 0.0,
            "z": 0.867,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 3.14,
        },
        "camera": {
            "x": 0.0,
            "y": 0.0,
            "z": 1.01,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "laser": {
            "x": 0.320,
            "y": 0.0,
            "z": 0.120,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 3.14,
        },
    }

    # Wheel positions
    WHEEL_POSITIONS = {
        "front_right_wheel": {
            # Since the module is symmetrical, x position is average of the four wheels x positions
            "x": 0.2548,
            "y": 0.17132,
            "z": 0.0025,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "front_left_wheel": {
            "x": -0.2548,
            "y": 0.17132,
            "z": 0.0025,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "back_left_wheel": {
            "x": -0.2548,
            "y": -0.17132,
            "z": 0.0025,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "back_right_wheel": {
            "x": 0.2548,
            "y": -0.17132,
            "z": 0.0025,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
    }


class EightInchRobotParameters:
    """Robot parameters like base plate are, wheel diameter, etc."""

    wheel_diameter = 0.203
    robot_length = 0.5
    robot_width = 0.482
    robot_height = 0.780
    wheel_base = 0.343  # Distance between wheel to wheel along x-axis
    track_width = 0.450  # Distance between wheel to wheel along y-axis

    min_rotation_rate = 1.0
    max_rotation_rate = 5.3  # wheel speed limit
    robot_velocity_limit = (
        4.3  # robot speed limit (patch: scaled down by a factor of 8)
    )

    threshold_range = 0.2  # Threshold for range sensor

    # Control Parameters for PID
    Kp = 1.0
    Kd = 0.1
    Ki = 0.1

    # Encoders parameters
    encoders_window_size = 3

    # The below parameters are for the modules and wheels with respect to the base frame
    # Base frame - Centre of the robot base plate.
    PUBLISH_RATE = {
        "imu": 50,
        "status": 0.2,
        "camera": 30,
        "lidar": 10,
        "ultrasonic": 10,
        "encoder": 30,
        "odometry": 30,  # Odometry rate would be the same as TF rate
    }

    ROBOT_BASE_FRAME = {"x": 0.6, "y": 0.50, "z": 0.2}
    # Sensor Positions
    SKIP_SENSORS: List[str] = []

    SENSOR_POSITIONS = {
        "imu_link": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.445,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "velodyne": {
            "x": 0.0,
            "y": 0.0,
            "z": 1.154,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "tof_front_left": {
            "x": 0.298,
            "y": 0.085,
            "z": 0.145,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.5235,
        },
        "tof_front_right": {
            "x": 0.298,
            "y": -0.085,
            "z": 0.154,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": -0.5235,
        },
        "tof_front_middle": {
            "x": 0.325,
            "y": 0.0,
            "z": 0.145,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "tof_right_middle": {
            "x": 0.179,
            "y": -0.173,
            "z": 0.145,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": -1.5708,
        },
        "tof_right_front": {
            "x": 0.250,
            "y": -0.150,
            "z": 0.145,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": -1.0472,
        },
        "tof_left_front": {
            "x": 0.250,
            "y": 0.150,
            "z": 0.145,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 1.0472,
        },
        "tof_left_middle": {
            "x": 0.179,
            "y": 0.173,
            "z": 0.145,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 1.5708,
        },
        "front_oakd_camera": {  # oak-d base frame
            "x": 0.065,
            "y": 0.0,
            "z": 0.867,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "left_oakd_camera": {
            "x": 0.0,
            "y": 0.065,
            "z": 0.867,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 1.57,
        },
        "right_oakd_camera": {
            "x": 0.0,
            "y": -0.065,
            "z": 0.867,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": -1.57,
        },
        "back_oakd_camera": {
            "x": -0.065,
            "y": 0.0,
            "z": 0.867,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 3.14,
        },
        "camera": {
            "x": 0.0,
            "y": 0.0,
            "z": 1.01,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "laser": {
            "x": 0.320,
            "y": 0.0,
            "z": 0.120,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 3.14,
        },
    }

    # Wheel positions
    WHEEL_POSITIONS = {
        "front_right_wheel": {
            # Since the module is symmetrical, x position is average of the four wheels x positions
            "x": 0.2548,
            "y": 0.17132,
            "z": 0.0025,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "front_left_wheel": {
            "x": -0.2548,
            "y": 0.17132,
            "z": 0.0025,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "back_left_wheel": {
            "x": -0.2548,
            "y": -0.17132,
            "z": 0.0025,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "back_right_wheel": {
            "x": 0.2548,
            "y": -0.17132,
            "z": 0.0025,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
    }


class TenInchRobotParameters:
    """Robot parameters like base plate are, wheel diameter, etc."""

    wheel_diameter = 0.267
    robot_length = 0.6
    robot_width = 0.50
    robot_height = 1.154
    wheel_base = 0.317  # Distance between wheel to wheel along x-axis
    track_width = 0.410  # Distance between wheel to wheel along y-axis

    min_rotation_rate = 1.0
    max_rotation_rate = 5.3  # wheel speed limit
    robot_velocity_limit = (
        4.3  # robot speed limit (patch: scaled down by a factor of 8)
    )

    threshold_range = 0.2  # Threshold for range sensor

    # Encoders parameters
    encoders_window_size = 3

    # Control Parameters for PID
    Kp = 1.0
    Kd = 0.001
    Ki = 0.1

    # The below parameters are for the modules and wheels with respect to the base frame
    # Base frame - Centre of the robot base plate.
    PUBLISH_RATE = {
        "imu": 50,
        "status": 0.2,
        "camera": 30,
        "lidar": 10,
        "range": 45,
        "encoder": 30,
        "odometry": 30,  # Odometry rate would be the same as TF rate
        "battery": 1,
        "environment": 1,
        "gas_detection": 1,
        "led_mode": 10,
    }

    ROBOT_BASE_FRAME = {"x": 0.6, "y": 0.50, "z": 0.2}
    # Sensor Positions

    SENSOR_POSITIONS = {
        "imu_link": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.535,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "camera": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.932,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "velodyne": {
            "x": 0.0,
            "y": 0.0,
            "z": 1.154,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "tof_front_left": {
            "x": 0.298,
            "y": 0.085,
            "z": 0.145,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.5235,
        },
        "tof_front_right": {
            "x": 0.298,
            "y": -0.085,
            "z": 0.154,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": -0.5235,
        },
        "tof_front_middle": {
            "x": 0.325,
            "y": 0.0,
            "z": 0.145,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "tof_right_middle": {
            "x": 0.179,
            "y": -0.173,
            "z": 0.145,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": -1.5708,
        },
        "tof_right_front": {
            "x": 0.250,
            "y": -0.150,
            "z": 0.145,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": -1.0472,
        },
        "tof_left_front": {
            "x": 0.250,
            "y": 0.150,
            "z": 0.145,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 1.0472,
        },
        "tof_left_middle": {
            "x": 0.179,
            "y": 0.173,
            "z": 0.145,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 1.5708,
        },
        "front_oakd_camera": {  # oak-d base frame
            "x": 0.065,
            "y": 0.0,
            "z": 0.867,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "left_oakd_camera": {
            "x": 0.0,
            "y": 0.065,
            "z": 0.867,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 1.57,
        },
        "right_oakd_camera": {
            "x": 0.0,
            "y": -0.065,
            "z": 0.867,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": -1.57,
        },
        "back_oakd_camera": {
            "x": -0.065,
            "y": 0.0,
            "z": 0.867,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 3.14,
        },
        "laser": {
            "x": 0.320,
            "y": 0.0,
            "z": 0.120,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 3.14,
        },
    }

    # Wheel positions
    WHEEL_POSITIONS = {
        "front_right_wheel": {
            # Since the module is symmetrical, x position is average of the four wheels x positions
            "x": 0.240,
            "y": 0.23,
            "z": 0.0025,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "front_left_wheel": {
            "x": -0.240,
            "y": 0.23,
            "z": 0.0025,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "back_left_wheel": {
            "x": -0.240,
            "y": -0.23,
            "z": 0.0025,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "back_right_wheel": {
            "x": 0.240,
            "y": -0.23,
            "z": 0.0025,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
    }


class NewRobotParameters:
    """Robot parameters like base plate are, wheel diameter, etc."""

    wheel_diameter = 0.203
    robot_length = 0.5
    robot_width = 0.482
    robot_height = 0.780
    wheel_base = 0.343  # Distance between wheel to wheel along x-axis
    track_width = 0.450  # Distance between wheel to wheel along y-axis

    min_rotation_rate = 1.0
    max_rotation_rate = 5.3  # wheel speed limit
    robot_velocity_limit = (
        4.3  # robot speed limit (patch: scaled down by a factor of 8)
    )

    threshold_range = 0.2  # Threshold for range sensor

    # Control Parameters for PID
    Kp = 1.0
    Kd = 0.1
    Ki = 0.1

    # Encoders parameters
    encoders_window_size = 3

    # The below parameters are for the modules and wheels with respect to the base frame
    # Base frame - Centre of the robot base plate.
    PUBLISH_RATE = {
        "imu": 50,
        "status": 0.2,
        "camera": 30,
        "lidar": 10,
        "ultrasonic": 10,
        "encoder": 30,
        "odometry": 30,  # Odometry rate would be the same as TF rate
    }

    ROBOT_BASE_FRAME = {"x": 0.6, "y": 0.50, "z": 0.2}
    # Sensor Positions

    SENSOR_POSITIONS = {
        "imu_link": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "velodyne": {
            "x": 0.293,
            "y": 0.0,
            "z": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "tof_front_left": {
            "x": 0.298,
            "y": 0.085,
            "z": 0.145,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.5235,
        },
        "tof_front_right": {
            "x": 0.298,
            "y": -0.085,
            "z": 0.154,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": -0.5235,
        },
        "tof_front_middle": {
            "x": 0.325,
            "y": 0.0,
            "z": 0.145,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "tof_right_middle": {
            "x": 0.179,
            "y": -0.173,
            "z": 0.145,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": -1.5708,
        },
        "tof_right_front": {
            "x": 0.250,
            "y": -0.150,
            "z": 0.145,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": -1.0472,
        },
        "tof_left_front": {
            "x": 0.250,
            "y": 0.150,
            "z": 0.145,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 1.0472,
        },
        "tof_left_middle": {
            "x": 0.179,
            "y": 0.173,
            "z": 0.145,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 1.5708,
        },
        "front_oakd_camera": {  # oak-d base frame
            "x": 0.065,
            "y": 0.0,
            "z": 0.867,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "left_oakd_camera": {
            "x": 0.0,
            "y": 0.065,
            "z": 0.867,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 1.57,
        },
        "right_oakd_camera": {
            "x": 0.0,
            "y": -0.065,
            "z": 0.867,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": -1.57,
        },
        "back_oakd_camera": {
            "x": -0.065,
            "y": 0.0,
            "z": 0.867,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 3.14,
        },
        "camera": {
            "x": 0.0,
            "y": 0.0,
            "z": 1.01,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "laser": {
            "x": 0.315,
            "y": 0.0,
            "z": 0.132,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 1.5708,
        },
    }

    # Wheel positions
    WHEEL_POSITIONS = {
        "front_right_wheel": {
            # Since the module is symmetrical, x position is average of the four wheels x positions
            "x": 0.2548,
            "y": 0.17132,
            "z": 0.0025,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "front_left_wheel": {
            "x": -0.2548,
            "y": 0.17132,
            "z": 0.0025,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "back_left_wheel": {
            "x": -0.2548,
            "y": -0.17132,
            "z": 0.0025,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "back_right_wheel": {
            "x": 0.2548,
            "y": -0.17132,
            "z": 0.0025,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
    }


SaurusRobotParameters = (
    BeastRobotParameters
) = LeukoRobotParameters = AjaiChicagoB1RobotParameters = NewRobotParameters
SimRobotParameters = EightInchRobotParameters  # Dummy Robot Parameters for Simulation
mini_robotRobotParameters = FourInchRobotParameters
