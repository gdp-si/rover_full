"""Different robot profiles based on different hardware configurations."""
import os
from typing import Tuple


class RobotProfileTemplate:
    # Sensors and Actuators
    VELODYNE: bool = False
    OAKD_ARRAY: bool = False
    RP_LIDAR: bool = False
    INTEL_REALSENSE: bool = False
    EIGHT_INCH_DRIVE_WHEELS: bool = False
    TEN_INCH_DRIVE_WHEELS: bool = False
    FOUR_INCH_DRIVE_WHEELS: bool = False
    BNO055: bool = False
    TOF_SENSOR_ARRAY: bool = False
    ARDUINO_IMU: bool = False
    ULTRASONIC_SENSOR_ARRAY: bool = False
    LED_ARRAY: bool = False
    SPEAKER: bool = False
    DISPLAY: bool = False
    ENVIRONMENT_SENSORS: bool = False
    SIM: bool = False

    # Processors
    XAVIER_AGX: bool = False
    ORIN_AGX: bool = False
    MINI_PC: bool = False

    # Camera
    # The camera xmin from front camera is at 45 degrees
    CAMERA_OFFSET: float = 45.0  # degrees (CONSTANT)
    # After cropping the image, the width of the image is 480 pixels
    FOV_AFTER_CROPPING: int = 480  # pixels
    FOV_BEFORE_CROPPING: int = 640  # pixels

    # Parameters list
    OAKD_ARRAY_LIST: Tuple[str] = tuple()  # type: ignore
    TOF_SENSOR_ARRAY_LIST: Tuple[str] = tuple()  # type: ignore
    ULTRASONIC_SENSOR_ARRAY_LIST: Tuple[str] = tuple()  # type: ignore

    # Navigation features
    NAVIGATION_KEEPOUT_ZONES: bool = False
    NAVIGATION_SPEED_LIMITS: bool = False
    ARUCO_ID: int = 0

    def __init__(self, name: str, **parameters):
        self.name = name

        assert (
            len(parameters) > 0
        ), "No parameters were passed to the RobotProfileTemplate constructor."

        assert (
            parameters.get("EIGHT_INCH_DRIVE_WHEELS")
            or parameters.get("TEN_INCH_DRIVE_WHEELS")
            or parameters.get("FOUR_INCH_DRIVE_WHEELS")
        ), "No drive wheels were specified in the RobotProfileTemplate constructor."

        assert (
            parameters.get("XAVIER_AGX")
            or parameters.get("ORIN_AGX")
            or parameters.get("MINI_PC")
        ), "No processors were specified in the RobotProfileTemplate constructor."

        if parameters.get("OAKD_ARRAY"):
            assert (
                parameters.get("OAKD_ARRAY_LIST") is not None
            ), "No OAK-D cameras were specified in the RobotProfileTemplate constructor."

        if parameters.get("TOF_SENSOR_ARRAY"):
            assert (
                parameters.get("TOF_SENSOR_ARRAY_LIST") is not None
            ), "No TOF sensors were specified in the RobotProfileTemplate constructor."

        if parameters.get("ULTRASONIC_SENSOR_ARRAY"):
            assert (
                parameters.get("ULTRASONIC_SENSOR_ARRAY_LIST") is not None
            ), "No ultrasonic sensors were specified in the RobotProfileTemplate constructor."

        self._set_parameters(parameters)

    def _set_parameters(self, parameters: dict):
        """Set parameters for configurations and other elements."""
        for key, value in parameters.items():
            if not hasattr(self, key):
                raise AttributeError(f"Invalid parameter: {key}")

            setattr(self, key, value)

    def __str__(self) -> str:
        return self.name

    def __repr__(self) -> str:
        return self.name

    def get_configurations(self) -> dict:
        """Return a dictionary of the profile considering only the true parameters."""
        values = {
            key: value
            for key, value in RobotProfileTemplate.__dict__.items()
            if key != "name" and "__" not in key
        }

        values.update(self.__dict__)

        return values


MINI_ROBOT = RobotProfileTemplate(
    "MINI_ROBOT",
    ORIN_AGX=True,
    VELODYNE=True,
    BNO055=True,
    FOUR_INCH_DRIVE_WHEELS=True,
    RP_LIDAR=False,
    SPEAKER=False,
    OAKD_ARRAY=False,
    OAKD_ARRAY_LIST=[
        "right_oakd_camera",
        "back_oakd_camera",
        "left_oakd_camera",
        "front_oakd_camera",
    ],
    ARUCO_ID=11,
    NAVIGATION_KEEPOUT_ZONES=False,
    NAVIGATION_SPEED_LIMITS=False,
)

BEAST = RobotProfileTemplate(
    "BEAST",
    MINI_PC=True,
    INTEL_REALSENSE=False,
    TOF_SENSOR_ARRAY=False,
    VELODYNE=True,
    ARDUINO_IMU=True,
    RP_LIDAR=True,
    LED_ARRAY=False,
    SPEAKER=False,
    TOF_SENSOR_ARRAY_LIST=(
        "tof_front_left",
        "tof_front_right",
        "tof_front_middle",
        "tof_right_middle",
        "tof_right_front",
        "tof_left_middle",
        "tof_left_front",
    ),
    OAKD_ARRAY=False,
    OAKD_ARRAY_LIST=(
        "right_oakd_camera",
        "back_oakd_camera",
        "left_oakd_camera",
        "front_oakd_camera",
    ),
    NAVIGATION_KEEPOUT_ZONES=True,
    NAVIGATION_SPEED_LIMITS=True,
    TEN_INCH_DRIVE_WHEELS=True,
    ARUCO_ID=12,
)


HOST = RobotProfileTemplate(
    "HOST",
    ORIN_AGX=True,
    VELODYNE=True,
    BNO055=True,
    RP_LIDAR=True,
    TOF_SENSOR_ARRAY=True,
    TOF_SENSOR_ARRAY_LIST=(
        "tof_front_left",
        "tof_front_right",
        "tof_front_middle",
        "tof_right_middle",
        "tof_right_front",
        "tof_left_middle",
        "tof_left_front",
    ),
    OAKD_ARRAY=False,
    OAKD_ARRAY_LIST=("front_oakd_camera"),
    EIGHT_INCH_DRIVE_WHEELS=True,
    NAVIGATION_KEEPOUT_ZONES=True,
    NAVIGATION_SPEED_LIMITS=True,
    ARUCO_ID=13,
)


SIM = RobotProfileTemplate(
    "SIM",
    MINI_PC=True,
    OAKD_ARRAY=False,
    VELODYNE=True,
    RP_LIDAR=True,
    OAKD_ARRAY_LIST=("front_oakd_camera",),
    ARUCO_ID=10,
    TEN_INCH_DRIVE_WHEELS=True,
    NAVIGATION_KEEPOUT_ZONES=True,
    NAVIGATION_SPEED_LIMITS=True,
    SIM=True,
)

SAURUS = RobotProfileTemplate(
    "SAURUS",
    MINI_PC=True,
    VELODYNE=True,
    ARDUINO_IMU=True,
    EIGHT_INCH_DRIVE_WHEELS=True,
    OAKD_ARRAY=True,
    LED_ARRAY=True,
    SPEAKER=True,
    OAKD_ARRAY_LIST=[
        "right_oakd_camera",
        "back_oakd_camera",
        "left_oakd_camera",
        "front_oakd_camera",
    ],
    RP_LIDAR=True,
    NAVIGATION_KEEPOUT_ZONES=True,
    NAVIGATION_SPEED_LIMITS=True,
    ARUCO_ID=10,
)

LEUKO = RobotProfileTemplate(
    "LEUKO",
    MINI_PC=True,
    VELODYNE=True,
    ARDUINO_IMU=True,
    EIGHT_INCH_DRIVE_WHEELS=True,
    OAKD_ARRAY=True,
    LED_ARRAY=True,
    SPEAKER=True,
    OAKD_ARRAY_LIST=[
        "front_oakd_camera",
        "right_oakd_camera",
        "back_oakd_camera",
        "left_oakd_camera",
    ],
    RP_LIDAR=True,
    NAVIGATION_KEEPOUT_ZONES=True,
    NAVIGATION_SPEED_LIMITS=True,
    ARUCO_ID=15,
)

AJAI_CHICAGO_B1 = RobotProfileTemplate(
    "AJAI_CHICAGO_B1",
    MINI_PC=True,
    VELODYNE=True,
    ARDUINO_IMU=True,
    EIGHT_INCH_DRIVE_WHEELS=True,
    OAKD_ARRAY=True,
    LED_ARRAY=True,
    SPEAKER=True,
    OAKD_ARRAY_LIST=[
        "front_oakd_camera",
        "right_oakd_camera",
        "back_oakd_camera",
        "left_oakd_camera",
    ],
    RP_LIDAR=True,
    NAVIGATION_KEEPOUT_ZONES=True,
    NAVIGATION_SPEED_LIMITS=False,
    ARUCO_ID=14,
)

# Get the environment variable for the robot profile
ROBOT_PROFILE = os.environ.get("ROBOT_PROFILE")
assert ROBOT_PROFILE is not None, "ROBOT_PROFILE environment variable not set"

# Set the robot profile
if ROBOT_PROFILE.lower() == "host":
    RobotProfile = HOST
elif ROBOT_PROFILE.lower() == "beast":
    RobotProfile = BEAST
elif ROBOT_PROFILE.lower() == "sim":
    RobotProfile = SIM
elif ROBOT_PROFILE.lower() == "saurus":
    RobotProfile = SAURUS
elif ROBOT_PROFILE.lower() == "leuko":
    RobotProfile = LEUKO
elif ROBOT_PROFILE.lower() == "mini_robot":
    RobotProfile = MINI_ROBOT
elif ROBOT_PROFILE.lower() == "ajai_chicago_b1":
    RobotProfile = AJAI_CHICAGO_B1
else:
    raise ValueError(f"Invalid robot profile: {ROBOT_PROFILE}")
