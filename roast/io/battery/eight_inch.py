"""Battery Class Definition"""
import os

import serial

from roast.glogging import Logger
from roast.io import SensorDefinition


def byt_int_conversion(data):
    """Byte to Integer Conversion

    Args:
        data (byte): Byte data

    Returns:
        int: Converted Integer
    """
    int_val = int.from_bytes(data, "big")
    return int_val


def convert_sec_to_hrs(seconds: int):
    """Convert seconds to hours

    Args:
        seconds (int): Seconds to convert

    Returns:
        tuple: Hours, Minutes and Seconds
    """
    hours = 0
    mins = 0
    # hours=seconds/3600

    seconds = seconds % (240 * 3600)
    hours = seconds // 3600
    seconds %= 3600
    mins = seconds // 60
    seconds %= 60
    return hours, mins, seconds


class Battery(SensorDefinition):
    """Battery Class Defintion ends here

    Args:
        SensorDefinition (ABC): Abstract class definition for Sensors

    Returns:
        dict: Battery Information
    """

    LOG = Logger(_module_name=os.path.basename(__file__))

    def __init__(self):
        self.config = EightInchBatteryParameters
        self.COM_PORT = serial.Serial(
            port=self.config.COM_PORT,
            baudrate=9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.2,
        )
        self.BATTERY_COMMAND = self.config.BATTERY_COMMAND

    def is_active(self):
        """Check if the battery is active

        Returns:
            bool: True if the battery is active observing battery current consumption
        """
        return bool(self._battery_currrent_consumption())

    def get_info(self):
        """Get the battery information

        Returns:
            str:Battery related information"""

        battery_ppercentage = self._battery_percentage()
        battery_ttemperature = self._battery_temperature()
        battery_vvoltage = self._battery_voltage()
        battery_ccapacity = self._battery_current_capacity()
        battery_cconsumption = self._battery_currrent_consumption()
        charging_time, discharging_time = self._battery_Charging_discharging_time()
        battery_charging_time, min1, sec1 = convert_sec_to_hrs(charging_time)
        battery_discharging_time, min2, sec2 = convert_sec_to_hrs(discharging_time)

        # Return the info
        return (
            f"Battery Percentage           = {battery_ppercentage} %\n"
            f"battery_temperature          = {battery_ttemperature} °C\n"
            f"Battery Voltage              = {battery_vvoltage} Volts\n"
            f"Battery Current Capacity     = {battery_ccapacity} Ah\n"
            f"battery_current_consumption  = {battery_cconsumption} A\n"
            f"Battery_charging_Time        = {battery_charging_time} Hrs:{min1} Min:{sec1} sec\n"
            f"Battery_Discharging_Time     = {battery_discharging_time} Hrs:{min2} Min:{sec2} sec\n"
        )

    def print_msg(self):
        """Prints theBattery information for debugging"""
        basic_info = self.get_info()

        # Get corresponding configurations
        config = str(self.config())

        self.LOG.INFO("Basic Battery Information: \n")
        print(basic_info)
        self.LOG.INFO(f" Battery configurations:\n {config}\n")

    # pylint: disable=arguments-differ
    def update(self):
        """Update the battery related information

        Returns:
            dict: Dictionary of battery related information in order
                - Battery_Percentage
                - Battery Temperature
                - Battery_Voltage
                - Battery_Current_Capacity
                - Battery_Current_Consumption
        """

        battery_ppercentage = self._battery_percentage()
        battery_ttemperature = self._battery_temperature()
        battery_vvoltage = self._battery_voltage()
        battery_ccapacity = self._battery_current_capacity()
        battery_consumption = self._battery_currrent_consumption()

        return {
            "battery_percentage": battery_ppercentage,
            "battery_temperature": battery_ttemperature,
            "battery_voltage": battery_vvoltage,
            "battery_current_capacity": battery_ccapacity,
            "battery_current_consumption": battery_consumption,
        }

    def _battery_current_capacity(self):  # Available Battery Current capacity
        comp_data = b"\xDD\x03\x00\x1B"
        Yes1 = self.COM_PORT.flush()
        Yes1 = self.COM_PORT.write(self.BATTERY_COMMAND)
        Yes1 = self.COM_PORT.readline()
        data_1 = Yes1

        z = data_1.find(comp_data)
        data_9 = data_1[z + 8 : 10]
        data_2 = data_1[z:]
        y = len(data_2)

        if y == 34:
            battery_ccapacity = (byt_int_conversion(data_9)) / 100
            return battery_ccapacity

        self.LOG.WARNING("Error in Reading Battery Current Capacity")
        return None

    def _battery_percentage(self):  # Battery Percentage
        comp_data = b"\xDD\x03\x00\x1B"
        Yes1 = self.COM_PORT.flush()
        Yes1 = self.COM_PORT.write(self.BATTERY_COMMAND)
        Yes1 = self.COM_PORT.readline()
        data_1 = Yes1
        z = data_1.find(comp_data)
        data_6 = data_1[z + 23 : 24]
        data_2 = data_1[z:]
        y = len(data_2)

        if y == 34:
            battery_percentage = byt_int_conversion(data_6)
            return battery_percentage

        self.LOG.WARNING("Error in Reading Battery Percentage")
        return None

    def _battery_voltage(self):  # Battery Voltage
        comp_data = b"\xDD\x03\x00\x1B"
        Yes1 = self.COM_PORT.flush()
        Yes1 = self.COM_PORT.write(self.BATTERY_COMMAND)
        Yes1 = self.COM_PORT.readline()
        data_1 = Yes1
        z = data_1.find(comp_data)

        data_3 = data_1[z + 4 : 6]
        data_2 = data_1[z:]
        y = len(data_2)

        if y == 34:
            battery_voltage = byt_int_conversion(data_3)
            battery_voltage /= 100
            return battery_voltage

        self.LOG.WARNING("Error in Reading Battery Voltage")
        return None

    def _battery_currrent_consumption(self):  # Current consumption based on usage
        comp_data = b"\xDD\x03\x00\x1B"
        Yes1 = self.COM_PORT.flush()
        Yes1 = self.COM_PORT.write(self.BATTERY_COMMAND)
        Yes1 = self.COM_PORT.readline()
        data_1 = Yes1
        z = data_1.find(comp_data)

        data_4 = data_1[z + 6 : 8]
        data_2 = data_1[z:]
        y = len(data_2)

        if y == 34:
            battery_current_consumption = (65536 - byt_int_conversion(data_4)) / 100
            if battery_current_consumption != 0:
                return battery_current_consumption

        self.LOG.WARNING("Error in Reading Battery Current Consumption")
        return None

    def _battery_temperature(self):  # Battery Temperature
        """Read Battery temperature

        Returns:
            float/None: Battery Temperature
        """
        comp_data = b"\xDD\x03\x00\x1B"
        Yes1 = self.COM_PORT.flush()
        Yes1 = self.COM_PORT.write(self.BATTERY_COMMAND)
        Yes1 = self.COM_PORT.readline()
        data_1 = Yes1
        z = data_1.find(comp_data)

        z = data_1.find(comp_data)
        data_7 = data_1[z + 29 : 31]
        data_2 = data_1[z:]
        y = len(data_2)

        if y == 34:
            battery_temperature = (byt_int_conversion(data_7) - 2731) / 10
            # Battery_Temperature=(Battery_Temperature-2731)/10
            # return "Battery Temperature       =", Battery_Temperature, "°C", "\n"
            return battery_temperature

        self.LOG.WARNING("Error in Reading Battery Temperature")
        return None

    def _battery_Charging_discharging_time(self):  # Battery Temperature
        comp_data = b"\xDD\x03\x00\x1B"
        Yes1 = self.COM_PORT.flush()
        Yes1 = self.COM_PORT.write(self.BATTERY_COMMAND)
        Yes1 = self.COM_PORT.readline()
        data_1 = Yes1
        z = data_1.find(comp_data)
        data_4 = data_1[z + 6 : 8]  # current consumption in Ampere
        data_9 = data_1[z + 8 : 10]  # actual Battery capacity in Ah
        # data_3 = data_1[z + 4 : 6]  # Battery voltage
        data_2 = data_1[z:]
        y = len(data_2)
        if y == 34:
            battery_current_consumption = (65536 - byt_int_conversion(data_4)) / 100
            # Battery_voltage = byt_int_conversion(data_3)/100
            battery_ccapacity = (byt_int_conversion(data_9)) / 100

            battery_charging_time = (
                86 - battery_ccapacity
            ) / 10  # Battery charger Supply current(10)
            battery_charging = battery_charging_time - int(battery_charging_time)
            hour1 = int(battery_charging_time - battery_charging)
            min1 = round(60 * battery_charging)
            charging_time = (hour1 * 60 * 60) + (min1 * 60)

            battery_discharging_time = battery_ccapacity / battery_current_consumption
            Battery_Discharging = battery_discharging_time - int(
                battery_discharging_time
            )
            hour2 = int(battery_discharging_time - Battery_Discharging)
            min2 = round(60 * Battery_Discharging)
            discharging_time = (hour2 * 60 * 60) + (min2 * 60)
            return charging_time, discharging_time

        self.LOG.WARNING("Error in battery parameters")
        return None, None
