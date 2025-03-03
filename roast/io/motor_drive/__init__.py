"""Motor Drive interface for 8-inch and 10-inch motors."""
from roast import RobotProfile

if RobotProfile.EIGHT_INCH_DRIVE_WHEELS:
    from roast.io.motor_drive.eight_inch import MotorControl  # type: ignore
elif RobotProfile.TEN_INCH_DRIVE_WHEELS:
    from roast.io.motor_drive.ten_inch import MotorControl  # type: ignore
elif RobotProfile.FOUR_INCH_DRIVE_WHEELS:
    from roast.io.motor_drive.four_inch import MotorControl  # type: ignore
else:
    raise ValueError(
        f"Unsupported Robot Profile for MotorControl: {RobotProfile.get_configurations()}"
    )

__all__ = ["MotorControl"]
