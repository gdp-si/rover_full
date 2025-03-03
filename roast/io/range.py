import serial

from roast.glogging import Logger
from roast.io.configs import RangeConfig
from roast.utils.patterns import ThreadSafeSingletonMetaclass


class Range(metaclass=ThreadSafeSingletonMetaclass):
    TOF_MAX_RANGE = RangeConfig.max_range
    TOF_MIN_RANGE = RangeConfig.min_range
    LOG = Logger(_module_name="Range Sensor")

    def __init__(self):
        self.config = RangeConfig()
        self.COM_PORT = serial.Serial(
            port=self.config.COM_PORT,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.2,
        )
        self.DEACTIVATE_STREAMING = b"\x00\x52\x02\x00\xD8"
        self.ACTIVATE_STREAMING = b"\x00\x52\x02\x01\xDF"
        self.TEXT_MODE = b"\x00\x11\x01\x45"
        self.BINARY_MODE = b"\x00\x11\x02\x4C"
        self.SIMULTANEOUS_MODE = b"\x00\x31\x01\xEB"
        self.SEQUENTIAL_MODE = b"\x00\x31\x02\xE2"
        self.DEACTIVATE_IMU = b"\x00\x41\x01\x49"
        self.HIGH_HZ = b"\x00\x52\x03\x02\xC3"  # 50 hz mode
        self.MAX_HZ = b"\x00\x52\x03\x06\xDF"
        self.ASAP_MODE = b"\x00\x52\x03\x01\xCA"
        self.QUATERNION_LINEAR_ACCELERATION_MODE = b"\x00\x41\x04\x52"
        self.EULER_MODE = b"\x00\x41\x03\x47"
        self.divisor = 2**14  # if using quaternion_linear_acc_mode need to use it

        # Initials
        self._right_middle = 0.0
        self._left_middle = 0.0
        self._left_front = 0.0
        self._front_left = 0.0
        self._front_middle = 0.0
        self._front_right = 0.0
        self._right_front = 0.0

        # Initials
        self._right_middle = 0.0
        self._left_middle = 0.0
        self._left_front = 0.0
        self._front_left = 0.0
        self._front_middle = 0.0
        self._front_right = 0.0
        self._right_front = 0.0

    def is_active(self):
        """Check if the Terabee sensor is Active or Not

        Returns:
            bool: True if the Terabee is active
        """
        return bool(self.COM_PORT.isOpen())

    def get_info(self):
        """Get the range related information
        Returns:
        str: range sensor related information"""
        return ""

    def print_msg(self):
        """Print the IMU related information for debugging"""
        basic_info = self.get_info()
        print(basic_info)

    def send_command(self, command):
        # with self.serial_lock:  # This avoid concurrent writes/reads of serial
        self.COM_PORT.write(command)

    def set_binary_mode(self):
        if self.send_command(self.BINARY_MODE):
            self.LOG.DEBUG("Sensor succesfully switched to binary mode")

    def activate_streaming(self):
        if self.send_command(self.ACTIVATE_STREAMING):
            self.LOG.DEBUG("Sensor succesfully started measuring measurement")

    def deactivate_streaming(self):
        if self.send_command(self.DEACTIVATE_STREAMING):
            self.LOG.DEBUG("Sensor succesfully deactivated measurements")

    def simultaneous_mode(self):
        if self.send_command(self.SIMULTANEOUS_MODE):
            self.LOG.DEBUG("Sensor succesfully switched to simultaneous mode")

    def text_mode(self):
        if self.send_command(self.TEXT_MODE):
            self.LOG.DEBUG("Sensor succesfully switched Text mode")

    def deactivate_imu(self):
        if self.send_command(self.DEACTIVATE_IMU):
            self.LOG.DEBUG("Deactivated  IMU measurement")

    def high_frquency(self):
        if self.send_command(self.HIGH_HZ):
            self.LOG.DEBUG("Sensor succesfully switched to desired frquency")

    def asap_frquency(self):
        if self.send_command(self.MAX_HZ):
            self.LOG.DEBUG("Sensor succesfully switched to desired frquency")

    def max_frquency(self):
        if self.send_command(self.ASAP_MODE):
            self.LOG.DEBUG("Sensor succesfully switched to desired frquency")

    def quaternion_linear_acceleration_mode(self):
        if self.send_command(self.QUATERNION_LINEAR_ACCELERATION_MODE):
            self.LOG.DEBUG("Sensor succesfully switched to quaternion_linear_acc mode")

    def euler_mode(self):
        if self.send_command(self.EULER_MODE):
            self.LOG.DEBUG("Sensor succesfully switched to euler mode")

    def tof_update(self):
        # self.deactivate_streaming()
        self.COM_PORT.flushInput()
        self.activate_streaming()
        self.text_mode()
        self.deactivate_imu()
        # self.high_frquency()
        while True:
            ack = str(self.COM_PORT.readline()).replace("\\r\\n'", "")
            ack = ack.replace("+Inf", "0.0")  # mm
            ack = ack.replace("-Inf", "3300")  # mm
            ack = ack.replace("-1", "0.0")  # mm
            arr = ack.split("\\t")

            try:
                self._right_middle = float(arr[1])
                self._left_middle = float(arr[2])
                self._left_front = float(arr[3])
                self._front_left = float(arr[4])
                self._front_middle = float(arr[5])
                self._front_right = float(arr[6])
                self._right_front = float(arr[7])
            except IndexError:
                self.LOG.WARNING("Skipping a data due to serialization problem")
            except ValueError:
                self.LOG.WARNING("Skipping invalid data")
            return {
                "tof_right_middle": self._right_middle / 1000,
                "tof_right_front": self._right_front / 1000,
                "tof_front_right": self._front_right / 1000,
                "tof_front_middle": self._front_middle / 1000,
                "tof_front_left": self._front_left / 1000,
                "tof_left_front": self._left_front / 1000,
                "tof_left_middle": self._left_middle / 1000,
            }

    def destroy(self):
        """Destroy the serial"""
        del self.COM_PORT

    # def imu_update(self,text : str):

    #     """
    #     this case for getting both imu and ToF data

    #     """
    #     self.COM_PORT.flushInput()
    #     self.activate_streaming()
    #     self.text_mode()
    #     self.euler_mode()

    #     if (text == "IM"):

    #         while True :
    #             # start_time = time.time()
    #             ack = str(self.COM_PORT.readline()).replace("\\r\\n'","")
    #             # ack = ack.replace("Inf","0")
    #             arr = ack.split('\\t')
    #             match arr[0]:
    #                 case "b'IM":
    #                     vals = [eval(i) for i in ar r[1:]]
    #                     self.roll= vals[1]/16
    #                     self.pitch = vals [2]/16
    #                     self.yaw = vals [0]/16
    #                     self.yaw=(360 - self.yaw) * 3.14159 / 180
    #                     self.yaw = wrap_angle(self.yaw)
    #                     # print("IMU -> "+str(vals))
    #                     # print(roll,pitch,self.yaw)
    #                     # elapsed_time = time.time() - start_time
    #                     # frequency = 1 / elapsed_time
    #                     # print("Frequency: {:.2f} Hz".format(frequency))
    #                     return {"euler":[self.roll,self.pitch,self.yaw]}
    # elif (text == "TH"):
    #     # start_time = time.time()

    #     while True :
    #         start_time = time.time()
    #         ack = str(self.COM_PORT.readline()).replace("\\r\\n'","")
    #         ack = ack.replace("Inf","0.0")
    #         # ack = ack.replace("-1","0.0")
    #         arr = ack.split('\\t')
    #         match arr[0]:

    #             case "b'TH":
    #                 vals = [eval(i) for i in arr[1:]]
    #                 print("TOF -> "+str(vals))
    #                 sensor_1 = vals[0]
    #                 sensor_2 = vals[1]
    #                 sensor_3 = vals[2]
    #                 sensor_4 = vals[3]
    #                 sensor_5 = vals[4]
    #                 sensor_6 = vals[5]
    #                 sensor_7 = vals[6]
    #                 elapsed_time = time.time() - start_time
    #                 frequency = 1 / elapsed_time
    #                 print("Frequency: {:.2f} Hz".format(frequency))

    #                 return sensor_1,sensor_2,sensor_3,sensor_4,sensor_5,sensor_6,sensor_7
