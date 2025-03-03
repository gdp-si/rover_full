"""Detect the ports connected to the robot automatically."""
from serial.tools import list_ports

from roast import RobotProfile

if RobotProfile.MINI_PC:
    PORTS = {
        "battery_driver": ("1-2.3", "/dev/null"),
        "gsm_module": ("1-4.4", "/dev/null"),
        "left_motor_drive": ("1-4.1.3.3", "/dev/null"),
        "right_motor_drive": ("1-4.1.3.2", "/dev/null"),
        "weapon_module": ("3-1.4.4.2", "/dev/null"),
        "left_ultrasonic_sensor": ("1-4.2.5", "/dev/null"),
        "right_ultrasonic_sensor": ("1-4.2.7", "/dev/null"),
        "front_ultrasonic_sensor": ("1-4.2.6", "/dev/null"),
        "lidar": ("3-1.2", "/dev/ttyrpl"),
        "imu": ("3-2", "/dev/null"),
    }
elif RobotProfile.ORIN_AGX:
    PORTS = {
        "battery_driver": ("1-2.3", "/dev/null"),
        "gsm_module": ("1-4.4", "/dev/null"),
        "left_motor_drive": ("1-4.1.3.3", "/dev/null"),
        "right_motor_drive": ("1-4.1.3.2", "/dev/null"),
        "threat_spray_module": ("1-2.2", "/dev/null"),
        "left_ultrasonic_sensor": ("1-4.2.5", "/dev/null"),
        "right_ultrasonic_sensor": ("1-4.2.7", "/dev/null"),
        "front_ultrasonic_sensor": ("1-4.2.6", "/dev/null"),
        "lidar": ("1-4.2", "/dev/ttyrpl"),
        "imu": ("3-4.2", "/dev/null"),
    }
else:
    raise SystemError("Port Parser not setup for AGX Xavier")


class PortParser:
    """Parse the port names."""

    _info = None

    def __call__(self):
        self._info = list_ports.comports()
        _port_keys = list(PORTS.keys())
        _port_locations = [loc for (loc, _) in PORTS.values()]

        for port in self._info:
            if port.location in _port_locations:
                index = _port_locations.index(port.location)
                key = _port_keys[index]
                PORTS[key] = (PORTS[key][0], port.device)

        return PORTS
