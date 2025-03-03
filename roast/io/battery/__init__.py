"""Motor Drive interface for 8-inch and 10-inch motors."""
import os

ROBOT_TYPE = os.getenv("ROBOT_TYPE", "8-inch")

if ROBOT_TYPE == "8-inch":
    from roast.io.battery.eight_inch import Battery  # type: ignore
elif ROBOT_TYPE == "10-inch":
    from roast.io.battery.ten_inch import Battery  # type: ignore

__all__ = ["Battery"]
