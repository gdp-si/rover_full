import time
from copy import deepcopy

import canalystii

from roast.utils.math import rotational_rate_to_rpm
from roast.utils.patterns import ThreadSafeSingletonMetaclass

# NOTE: Manual change in res will create problem
res = []  # type: ignore


class TenInchMotorDriver(metaclass=ThreadSafeSingletonMetaclass):
    _driver = None  # NOTE: Manual change in driver will create problem

    def __init__(self):
        TenInchMotorDriver._driver = canalystii.CanalystDevice(
            bitrate=500000, timing0=None, timing1=None
        )


def int_to_byte(number):
    if number >= 0:
        num1 = int(number)
        bytes_val1 = num1.to_bytes(4, "big")
        bytes_val1 = int.from_bytes(bytes_val1, "big")
        hexadecimal_result = format(bytes_val1, "07X")

        res[:] = hexadecimal_result.zfill(8)

        orderdata = (
            res[6] + res[7] + res[4] + res[5] + res[2] + res[3] + res[0] + res[1]
        )
        return orderdata
    elif number < 0:
        signed_int = number

        unsigned_int = signed_int + 2**32

        bytes_val1 = unsigned_int.to_bytes(4, "little")
        bytes_val1 = int.from_bytes(bytes_val1, "little")
        hexadecimal_result = format(bytes_val1, "07X")

        res[:] = hexadecimal_result.zfill(8)

        orderdata = (
            res[6] + res[7] + res[4] + res[5] + res[2] + res[3] + res[0] + res[1]
        )
        return orderdata

    else:
        print(" wrong input")


def Motorcontroldata(Speed):
    """
    Converts Speed int value to Hex
    """
    pulseconv = int_to_byte(Speed)
    const = "23FF6000"
    fulldata = const + pulseconv

    i = tuple(list(fulldata))

    hexdata = (
        "0x"
        + i[0]
        + i[1]
        + ","
        + "0x"
        + i[2]
        + i[3]
        + ","
        + "0x"
        + i[4]
        + i[5]
        + ","
        + "0x"
        + i[6]
        + i[7]
        + ","
        + "0x"
        + i[8]
        + i[9]
        + ","
        + "0x"
        + i[10]
        + i[11]
        + ","
        + "0x"
        + i[12]
        + i[13]
        + ","
        + "0x"
        + i[14]
        + i[15]
    )
    return eval(hexdata)


class MotorControl:
    """Motor Control Definition for the Motor"""

    POSITIONMODE = None
    VELOCITYMODE = None
    DISABLE = None
    ENABLE = None
    ACCELSPEED = None
    DIACCELSPEED = None
    START = None
    DRIVERTEMPERATURE = None
    MOTORTEMPERATURE = None
    MOTORSTATUS = None
    HALLSTATUS = None
    ACTUALSPEEDFEEDBACK = None
    ACTUALPOSITIONFEEDBACK = None
    CLEARPOSFEEDBACK = None

    def __init__(self, motor_side: str, config: dict):
        self.motor_side = motor_side
        self.config = config

        self._get_hex_codes()

        self.device_id = deepcopy(self.config["device_id"][self.motor_side])

        self.Channels = deepcopy(self.config["channel_port"])
        self.Direction_factor = deepcopy(self.config["direction_factor"])
        self.desired_rpm = 0
        self.driver = TenInchMotorDriver()._driver

    def _get_hex_codes(self):
        self.POSITIONMODE = (
            0x2F,
            0x60,
            0x60,
            0x00,
            0x01,
            0x00,
            0x00,
            0x00,
        )  # POSITION MODE SET HEX CODE
        self.VELOCITYMODE = (
            0x2F,
            0x60,
            0x60,
            0x00,
            0x03,
            0x00,
            0x00,
            0x00,
        )  # VELOCITY MODE SET HEX CODE
        self.DISABLE = (
            0x2B,
            0x40,
            0x60,
            0x00,
            0x06,
            0x00,
            0x00,
            0x00,
        )  # DISABLE SET HEX CODE
        self.ENABLE = (
            0x2B,
            0x40,
            0x60,
            0x00,
            0x07,
            0x00,
            0x00,
            0x00,
        )  # ENEBLE HEX CODE
        self.ACCELSPEED = (
            0x23,
            0x83,
            0x60,
            0x00,
            0x64,
            0x00,
            0x00,
            0x00,
        )  # ACCELARATION TIME SET HEX CODE
        self.DIACCELSPEED = (
            0x23,
            0x84,
            0x60,
            0x00,
            0x64,
            0x00,
            0x00,
            0x00,
        )  # DEACCELERATION TIME SET HEX CODE
        self.START = (
            0x2B,
            0x40,
            0x60,
            0x00,
            0x0F,
            0x00,
            0x00,
            0x00,
        )  # START COMMAND HEX CODE
        self.DRIVERTEMPERATURE = (
            0x4B,
            0x26,
            0x20,
            0x02,
            0x00,
            0x00,
            0x00,
            0x00,
        )  # DRIVER TEMPERATURE FEEDBACK HEX CODE
        self.MOTORTEMPERATURE = (
            0x4B,
            0x26,
            0x20,
            0x01,
            0x00,
            0x00,
            0x00,
            0x00,
        )  # MOTOR TEMPERATURE FEEDBACK HEX CODE
        self.MOTORSTATUS = (
            0x4B,
            0x27,
            0x20,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
        )  # MOTOR SHAFT STATUS
        self.HALLSTATUS = (
            0x4B,
            0x28,
            0x20,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
        )  # ENCODER WIRE PHYSICAL CONNECTION STATUS
        self.ACTUALSPEEDFEEDBACK = (
            0x4B,
            0x6C,
            0x60,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
        )  # ACTUAL SPEED FEEDBACK HEX CODE
        self.ACTUALPOSITIONFEEDBACK = (
            0x43,
            0x64,
            0x60,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
        )  # ACTUAL POSITION FEEDBACK HEX CODE
        self.CLEARPOSFEEDBACK = (
            0x2B,
            0x05,
            0x20,
            0x00,
            0x01,
            0x00,
            0x00,
            0x00,
        )  # CLEAR PULSE HEX CODE

    def enable_motor(self):
        """Enable motor driver

        Returns:

        bool: True if the motor is enabled else False
        """

        self.write(self.VELOCITYMODE, self.device_id)
        self.write(self.DISABLE, self.device_id)
        self.write(self.ENABLE, self.device_id)

        self.write(self.ACCELSPEED, self.device_id)
        self.write(self.DIACCELSPEED, self.device_id)
        self.write(self.START, self.device_id)

        self.write(self.CLEARPOSFEEDBACK, self.device_id)

        time.sleep(0.001)

    def is_active(self):
        """Check wheather  Motor is active or not
        Returns:
        True<bool> : for active
        False<bool>: Not  Active
            int: Motor_Driver Temperature
        """

        return True

    def clear_encoder(self):
        self.write(self.CLEARPOSFEEDBACK, self.device_id)

    def update(self, left: float = 0.0, right: float = 0.0):
        ldesired_rpm = int(rotational_rate_to_rpm(left))
        rdesired_rpm = int(rotational_rate_to_rpm(right))

        ldesired_rpm = -1 * ldesired_rpm
        rdesired_rpm = 1 * rdesired_rpm

        l_motordata = Motorcontroldata(ldesired_rpm)
        r_motordata = Motorcontroldata(rdesired_rpm)

        self.write(r_motordata, 0x601)
        self.write(l_motordata, 0x603)
        time.sleep(0.001)

    def rpulse_counter(self):
        """
        Reads the ActualPositionfeedback  register from driver,

        return the PulseCount of Motor in Int Value
        """
        self.write(self.ACTUALPOSITIONFEEDBACK, self.device_id)
        stat = self.driver.receive(self.Channels)
        length = len(stat)
        if length > 0:
            k = stat[length - 1]
            stat = str(k)
            stat = stat.split()
            k = list(stat[3])
            k = k[13:]
            k = (str(k[6] + k[7] + k[4] + k[5] + k[2] + k[3] + k[0] + k[1])).encode()
            k = int(k, 16)
            if k & 0x80000000 == 0x80000000:
                k = 0xFFFFFFFF - k
                dist = (k / 16385) * (3.14 * 0.267)
                return -dist
            else:
                dist = (k / 16385) * (3.14 * 0.267)
                return dist
        else:
            return 0.0

    def lpulse_counter(self):
        """
        Reads the ActualPositionfeedback  register from driver,

        return the PulseCount of Motor in Int Value
        """
        self.write(self.ACTUALPOSITIONFEEDBACK, self.device_id)
        stat = self.driver.receive(self.Channels)
        length = len(stat)
        if length > 0:
            k = stat[length - 1]
            stat = str(k)
            stat = stat.split()
            k = list(stat[3])
            k = k[13:]
            k = (str(k[6] + k[7] + k[4] + k[5] + k[2] + k[3] + k[0] + k[1])).encode()
            k = int(k, 16)
            if k & 0x80000000 == 0x80000000:
                k = 0xFFFFFFFF - k
                dist = (k / 16385) * (3.14 * 0.267)
                return dist
            else:
                dist = (k / 16385) * (3.14 * 0.267)
                return -dist
        else:
            return 0.0

    def lget_wheels_travelled(self):
        m = self.lpulse_counter()
        return m

    def rget_wheels_travelled(self):
        m = self.rpulse_counter()
        return m

    def write(self, outdata, DEVICE_ID):
        """
        Transmit the
        """
        msg = canalystii.Message(
            can_id=DEVICE_ID, remote=False, extended=False, data_len=8, data=outdata
        )
        try:
            self.driver.send(self.Channels, [msg] * 4)
            time.sleep(0.001)
        except Exception as e:
            print("Error in write", e)

    def destroy(self):
        self.driver.stop(self.Channels)

        # Close the USB device
        self.driver._dev.reset()
