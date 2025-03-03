import time
from copy import deepcopy
from typing import List

import canalystii

from roast.glogging import Logger
from roast.io.configs import MotorDriveConfig
from roast.utils.math import rotational_rate_to_rpm
from roast.utils.patterns import ThreadSafeSingletonMetaclass

res: List[str] = []


class FourInchMotorDriver(metaclass=ThreadSafeSingletonMetaclass):
    _driver = None  # NOTE: Manual change in driver will create problem

    def __init__(self):
        FourInchMotorDriver._driver = canalystii.CanalystDevice(
            bitrate=500000, timing0=None, timing1=None
        )


def int_to_byte(number):
    if number >= 0:
        num1 = int(number)
        bytes_val1 = num1.to_bytes(4, "big")
        bytes_val1 = int.from_bytes(bytes_val1, "big")
        hexadecimal_result = format(bytes_val1, "07X")
        res[:] = hexadecimal_result.zfill(8)
        # print(res[:])
        orderdata = res[6] + res[7] + res[4] + res[5]
        return orderdata
    elif number < 0:
        signed_int = number
        unsigned_int = signed_int + 2**32
        bytes_val1 = unsigned_int.to_bytes(4, "little")
        bytes_val1 = int.from_bytes(bytes_val1, "little")
        hexadecimal_result = format(bytes_val1, "07X")
        res[:] = hexadecimal_result.zfill(8)
        orderdata = res[6] + res[7] + res[4] + res[5]
        return orderdata

    else:
        print(" wrong input")


def LeftMotorcontroldata(Speed):
    """
    Converts Speed int value to Hex
    """
    pulseconv = int_to_byte(Speed)
    const = "23FF6001"
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


def RightMotorcontroldata(Speed):
    """
    Converts Speed int value to Hex
    """
    pulseconv = int_to_byte(Speed)
    # print(pulseconv)
    const = "23FF6002"
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


def Motorcontroldata(Speed, R_Speed):
    """
    Converts Speed int value to Hex
    """
    pulseconv = int_to_byte(Speed)
    pulseconv1 = int_to_byte(R_Speed)
    const = "23FF6003"
    fulldata = const + pulseconv + pulseconv1

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
    # print(hexdata)
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
    LEFTACTUALPOSITIONFEEDBACK = None
    RIGHTACTUALPOSITIONFEEDBACK = None

    CLEARPOSFEEDBACK = None
    SYNCHRONIZATION = None
    LEFTMOTORACCELERATION = None
    RIGHTMOTORACCELERATION = None
    LEFTMOTORDECELERATION = None
    RIGHTMOTORDECELERATION = None
    ASYNCHRONOUS = None
    LEFTMOTORRPM = None
    RIGHTMOTORRPM = None

    def __init__(self, motor_side: str, config: dict):
        # print(motor_side)
        self.motor_side = motor_side
        self.config = config

        self._get_hex_codes()

        self.device_id = deepcopy(self.config["device_id"][self.motor_side])
        # print(self.device_id)
        self.Channels = deepcopy(self.config["channel_port"])
        self.Direction_factor = deepcopy(self.config["direction_factor"])
        self.desired_rpm = 0
        self.driver = FourInchMotorDriver()._driver

    def _get_hex_codes(self):
        self.SYNCHRONIZATION = (0x2B, 0x0F, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00)
        self.VELOCITYMODE = (0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00)
        self.LEFTMOTORACCELERATION = (0x23, 0x83, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00)
        self.RIGHTMOTORACCELERATION = (0x23, 0x83, 0x60, 0x02, 0x64, 0x00, 0x00, 0x00)
        self.LEFTMOTORDECELERATION = (0x23, 0x84, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00)
        self.RIGHTMOTORDECELERATION = (0x23, 0x84, 0x60, 0x02, 0x64, 0x00, 0x00, 0x00)
        self.ENABLE = (0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00)
        self.DISABLE = (0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00)
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
        # self.LEFTMOTORRPM=(23 FF 60 01 64 00 00 00 )
        # self.LEFTMOTORRPM=(23 FF 60 01 64 00 00 00 )
        self.ASYNCHRONOUS = (0x2B, 0x0F, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00)

        self.LEFTACTUALPOSITIONFEEDBACK = (
            0x43,
            0x64,
            0x60,
            0x01,
            0x00,
            0x00,
            0x00,
            0x00,
        )
        self.RIGHTACTUALPOSITIONFEEDBACK = (
            0x43,
            0x64,
            0x60,
            0x02,
            0x00,
            0x00,
            0x00,
            0x00,
        )

        self.CLEARPOSFEEDBACK = (
            0x2B,
            0x05,
            0x20,
            0x00,
            0x03,
            0x00,
            0x00,
            0x00,
        )  # CLEAR PULSE HEX CODE

    def enable_motor(self):
        """Enable motor driver

        Returns:

        bool: True if the motor is enabled else False
        """
        # print("cskkb")
        # self.write(self.ASYNCHRONOUS,self.device_id)
        self.write(self.SYNCHRONIZATION, self.device_id)
        self.write(self.VELOCITYMODE, self.device_id)
        self.write(self.DISABLE, self.device_id)
        self.write(self.ENABLE, self.device_id)

        self.write(self.LEFTMOTORACCELERATION, self.device_id)
        self.write(self.RIGHTMOTORACCELERATION, self.device_id)
        self.write(self.LEFTMOTORDECELERATION, self.device_id)
        self.write(self.RIGHTMOTORDECELERATION, self.device_id)

        self.write(self.START, self.device_id)
        self.write(self.CLEARPOSFEEDBACK, self.device_id)
        # print(self.device_id,self.motor_side)
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
        # print("clear",self.device_id,self.motor_side)
        self.write(self.CLEARPOSFEEDBACK, self.device_id)

    def update(self, left: float = 0.0, right: float = 0.0):
        ldesired_rpm = int(rotational_rate_to_rpm(left))
        rdesired_rpm = int(rotational_rate_to_rpm(right))

        ldesired_rpm = 1 * ldesired_rpm
        rdesired_rpm = -1 * rdesired_rpm

        # print(ldesired_rpm,rdesired_rpm)
        motordata = Motorcontroldata(ldesired_rpm, rdesired_rpm)

        self.write(motordata, self.device_id)

        time.sleep(0.001)

    def lpulse_counter(self):
        """
        Reads the ActualPositionfeedback  register from driver,

        return the PulseCount of Motor in Int Value
        """
        self.write(self.LEFTACTUALPOSITIONFEEDBACK, self.device_id)
        stat = self.driver.receive(self.Channels)
        # print(stat)
        length = len(stat)
        # print(l)
        if length > 0:
            k = stat[length - 1]
            stat = str(k)
            stat = stat.split()
            k = list(stat[3])
            # print(k)
            k = k[13:]
            # print(k)
            k = (str(k[6] + k[7] + k[4] + k[5] + k[2] + k[3] + k[0] + k[1])).encode()
            k = int(k, 16)
            # print("left",k)
            if k & 0x80000000 == 0x80000000:
                k = 0xFFFFFFFF - k
                dist = (k / 4096) * (0.34)
                # print(dist)
                return -dist
            else:
                dist = (k / 4096) * (0.34)
                # print(dist)
                return dist
        else:
            return 0.0

    def rpulse_counter(self):
        """
        Reads the ActualPositionfeedback  register from driver,

        return the PulseCount of Motor in Int Value
        """
        self.write(self.RIGHTACTUALPOSITIONFEEDBACK, self.device_id)
        stat = self.driver.receive(self.Channels)
        # print(stat)
        length = len(stat)
        # print(l)
        if length > 0:
            k = stat[length - 1]
            stat = str(k)
            stat = stat.split()
            k = list(stat[3])
            # print(k)
            k = k[13:]
            # print(k)
            k = (str(k[6] + k[7] + k[4] + k[5] + k[2] + k[3] + k[0] + k[1])).encode()
            k = int(k, 16)
            # print(k)
            # print("right",k)
            if k & 0x80000000 == 0x80000000:
                k = 0xFFFFFFFF - k
                # print("right",k)
                dist = (k / 4096) * (0.34)
                # print(dist)
                return dist
            else:
                dist = (k / 4096) * (0.34)
                # print(dist)
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
