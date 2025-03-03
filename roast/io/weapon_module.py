import time
from threading import Thread
from typing import Optional

import serial

from roast.glogging import Logger
from roast.io.port_parser import PortParser
from roast.utils.patterns import ThreadSafeSingletonMetaclass

PORTS = PortParser()()


class ThreatSprayModule(metaclass=ThreadSafeSingletonMetaclass):
    LOG = Logger("Threat Spray Module")

    def __init__(self) -> None:
        self.device = serial.Serial(
            port=PORTS["weapon_module"][1], baudrate=115200, timeout=0.2
        )
        self._retraction_timeout = 1.0
        self._thread: Optional[Thread] = None
        self._threat_detected = False

    @property
    def retraction_timeout(self):
        return self._retraction_timeout

    @retraction_timeout.setter
    def retraction_timeout(self, value):
        self._retraction_timeout = value

    def is_active(self):
        """Check if the motor drive is active."""
        return bool(self.device.isOpen())

    def update(self, command) -> None:
        self._threat_detected = command == "threat"

        if self._threat_detected:
            self._thread = Thread(target=self.threat_detection, daemon=True)
            self._thread.start()
            return
        else:
            if self._thread is not None:
                self._thread.join()
            self.device.write(b"off\n")

            # Time to allow serial write
            time.sleep(1.0)

        if command == "stop":
            self.device.write(b"red\n")
        elif command == "move":
            self.device.write(b"green\n")
        elif command == "off":
            self.device.write(b"off\n")

    def __del__(self):
        self.device.write(b"off\n")

    def threat_detection(self):
        while self._threat_detected:
            self.device.write(b"blink\n")
            time.sleep(0.1)
