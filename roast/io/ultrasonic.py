import adafruit_us100
import serial

from roast.glogging import Logger
from roast.io.configs import UltraConfig
from roast.utils.patterns import ThreadSafeSingletonMetaclass


class Ultrasonic(metaclass=ThreadSafeSingletonMetaclass):
    ULTRA_MAX_RANGE = UltraConfig.max_range
    ULTRA_MIN_RANGE = UltraConfig.min_range

    LOG = Logger(_module_name="Ultrasonic Sensor")

    def __init__(self, ultra_side: str):
        self.config = UltraConfig()
        self.Ultra_side = ultra_side
        self.COM_PORT = serial.Serial(
            port=self.config.ports[ultra_side], baudrate=9600, timeout=0.2
        )

    def is_active(self):
        """Check if the Ultrasonic sensor is Active or Not

        Returns:
            bool: True if the Ultrasonic  is active
        """
        return bool(self.COM_PORT.isOpen())

    def __str__(self) -> str:
        return self.Ultra_side

    def get_info(self):
        pass

    def update(self):
        us100 = adafruit_us100.US100(self.COM_PORT)
        distance = us100.distance / 100

        if distance > UltraConfig.max_range:
            return UltraConfig.max_range
        elif distance < UltraConfig.min_range:
            return UltraConfig.min_range
        # print(distance)

        return distance


# if __name__ =="__main__":
#     a=Ultrasonic("front_sensor")
#     while True:
#         a.update()
