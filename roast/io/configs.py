"""Configurations for Roast IO"""
from typing import Dict

from roast import RobotProfile
from roast.io.port_parser import PortParser
from roast.utils import compare_tuple

# pylint: disable=too-few-public-methods

PORTS = PortParser()()


class UltraConfig:
    """Ultrasonic sensor configuration"""

    ULTRASOUND = 1
    radiation_type = "infrared"
    field_of_view = 0.261799  # 15 degrees in radians
    min_range = 0.03
    max_range = 4.5
    fixed_range = 1.3
    mode = "VARIABLE"  # "FIXED" or "VARIABLE"

    ports = {
        "left": PORTS["left_ultrasonic_sensor"][1],
        "front": PORTS["front_ultrasonic_sensor"][1],
        "right": PORTS["right_ultrasonic_sensor"][1],
    }


class MotorDriveConfig:
    PARAMS_LEFT = {}
    PARAMS_RIGHT = {}
    if RobotProfile.EIGHT_INCH_DRIVE_WHEELS:
        """8 inch Motor Driver Configs for both drivers"""
        PARAMS_LEFT = {
            "device_id": {"left": 0x601},
            "channel_port": 0,
            "direction_factor": 1,
        }

        PARAMS_RIGHT = {
            "device_id": {"right": 0x601},
            "channel_port": 0,
            "direction_factor": 1,
        }
    elif RobotProfile.FOUR_INCH_DRIVE_WHEELS:
        """8 inch Motor Driver Configs for both drivers"""
        PARAMS_LEFT = {
            "device_id": {"left": 0x601},
            "channel_port": 0,
            "direction_factor": 1,
        }

        PARAMS_RIGHT = {
            "device_id": {"right": 0x601},
            "channel_port": 0,
            "direction_factor": 1,
        }
    elif RobotProfile.TEN_INCH_DRIVE_WHEELS:
        """10 inch Motor Driver Configs for both drivers"""
        PARAMS_LEFT = {
            "device_id": {"left": 0x603},
            "channel_port": 0,
            "direction_factor": -1,
        }

        PARAMS_RIGHT = {
            "device_id": {"right": 0x601},
            "channel_port": 0,
            "direction_factor": 1,
        }
    else:
        raise NotImplementedError(
            f"Unsupported Robot Profile for MotorDriveConfigs: {RobotProfile.get_configurations()}"
        )

    assert (
        len(PARAMS_LEFT) > 0
    ), "MotorDriveConfigs are not defined for this robot profile"
    assert (
        len(PARAMS_RIGHT) > 0
    ), "MotorDriveConfigs are not defined for this robot profile"


class ImuConfig:
    """IMU Related Configurations"""

    I2C_ADDR = 0x28
    BUS = None
    if RobotProfile.XAVIER_AGX:
        BUS = 8
    elif RobotProfile.ORIN_AGX:
        BUS = 7
    else:
        if RobotProfile.BNO055:
            raise ValueError(
                f"Unsupported Robot Profile for ImuConfig: {RobotProfile.get_configurations()}"
            )

    if RobotProfile.BNO055:
        assert BUS is not None, "IMU Bus is not defined for this robot profile"


class RangeConfig:
    """Range sensor configuration"""

    ULTRASOUND = 1
    radiation_type = "infrared"
    field_of_view = 0.2617994  # 15 degrees in radians
    min_range = 0.03
    max_range = 3.3
    fixed_range = 1.3
    mode = "VARIABLE"  # "FIXED" or "VARIABLE"
    COM_PORT = "/dev/ttyACM0"


class OAKDConfigs:
    device_ids: Dict[str, str] = {}
    if RobotProfile.name == "HOST":
        device_ids = {
            # "front_oakd_camera": "",
            # "right_oakd_camera": "",
            # "back_oakd_camera": "",
            # "left_oakd_camera": "",
        }
    elif RobotProfile.name == "BEAST":
        device_ids = {
            "right_oakd_camera": "194430101140481300",
            "back_oakd_camera": "19443010216C771300",
            "left_oakd_camera": "194430107146481300",
            "front_oakd_camera": "194430105180481300",
        }
    elif RobotProfile.name == "SIM":
        device_ids = {
            "front_oakd_camera": "184430106147AA0F00",  # Selva's Camera
        }
    elif RobotProfile.name == "SAURUS":
        device_ids = {
            "right_oakd_camera": "1944301001DB4D1300",
            "back_oakd_camera": "19443010616F771300",
            "left_oakd_camera": "194430106196771300",
            "front_oakd_camera": "19443010F168771300",
        }
    elif RobotProfile.name == "MINI_ROBOT":
        device_ids = {
            # "right_oakd_camera": "",
            # "back_oakd_camera": "",
            # "left_oakd_camera": "",
            # "front_oakd_camera": "",
        }
    elif RobotProfile.name == "LEUKO":
        device_ids = {
            "front_oakd_camera": "19443010E1F7771300",
            "right_oakd_camera": "194430101169771300",
            "back_oakd_camera": "19443010A158771300",
            "left_oakd_camera": "19443010710B781300",
        }
    elif RobotProfile.name == "AJAI_CHICAGO_B1":
        device_ids = {
            "front_oakd_camera": "18443010214F9A0F00",
            "right_oakd_camera": "18443010E129371300",
            "back_oakd_camera": "19443010713D841300",
            "left_oakd_camera": "19443010617C771300",
        }

    if RobotProfile.OAKD_ARRAY:
        assert (
            len(device_ids) > 0
        ), "OAKD Configs are not defined for this robot profile"
        assert compare_tuple(
            RobotProfile.OAKD_ARRAY_LIST, tuple(device_ids.keys())
        ), f"OAKD Configs are not well-defined for this robot profile: {RobotProfile.name}"
