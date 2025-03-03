"""Battery Class Definition"""
import os
from copy import deepcopy

from pymodbus.client.sync import ModbusSerialClient as ModbusClient

from roast.glogging import Logger
from roast.utils.patterns import ThreadSafeSingletonMetaclass


class Battery(metaclass=ThreadSafeSingletonMetaclass):
    """Battery Class Defintion ends here

    Args:
        SensorDefinition (ABC): Abstract class definition for Sensors

    Returns:
        dict: Battery Information
    """

    LOG = Logger(_module_name=os.path.basename(__file__))

    def __init__(self, port="/dev/ttyUSB0"):
        self.config = TenInchBatteryParameters()
        self._port = self.config.port
        self.client = ModbusClient(
            method="rtu", port=self._port, baudrate=9600, timeout=0x0001
        )
        self.client.connect()
        self.ID = 0x0001
        self.BATTERY = 0x000A
        self.Charger_Capacity = 20

    def _get_config(self):
        """Get the config for the Battery."""
        config = TenInchBatteryParameters()

        return {
            "port": deepcopy(config.port),
        }

    def modbus_fail_read_handler(self, ADDR, WORD):
        """Modbus Fail Read Handler"""
        read_success = False
        reg = [None] * WORD
        while not read_success:
            result = self.client.read_holding_registers(ADDR, WORD, unit=self.ID)
            try:
                for i in range(WORD):
                    reg[i] = result.registers[i]
                read_success = True
            except AttributeError as e:
                self.LOG.ERROR(f"Modbus Read Failed, {e}")

        return reg

    def is_active(self) -> bool:
        """Check if the motor drive is active."""
        return bool(self.client.connect())

    def get_info(self):
        """Get the battery information

        Returns:
            str:Battery related information"""
        battery_data = self.update()

        return f"Battery Data = {battery_data} %\n"

    def print_msg(self):
        """Prints theBattery information for debugging"""
        # basic_info = self.get_info()

        # Get corresponding configurations
        config = str(self.update())

        self.LOG.INFO("Basic Battery Information: \n")
        self.LOG.INFO(f" Battery configurations:\n {config}\n")

    def update(self):  # pylint: disable=args-differ
        """Update the battery information"""
        registers = self.modbus_fail_read_handler(self.BATTERY, 0x00016)
        # print(registers)
        ticks = registers[0]
        voltage = registers[2] / 1000
        Current_Consumption = registers[4] / 100
        Maximum_Voltage = registers[8] / 1000
        Minimum_Voltage = registers[9] / 1000
        Battery_Percentage = registers[10] * 0.4
        Battery_Capacity = registers[11] / 10
        Watt_Hour = (registers[2] * registers[4]) / 100000
        Remaining_Capacity = Battery_Capacity * (Battery_Percentage / 100)
        Charge_Time = int(
            ((Battery_Capacity - Remaining_Capacity) / self.Charger_Capacity) * 3600
        )
        Remaining_Run_Time = int(Remaining_Capacity / Current_Consumption) * 3600
        Temp = registers[6] >> 8
        Temp = Temp - 40
        battery_cell_voltage1 = float((registers[13] / 1000))
        battery_cell_voltage2 = float(registers[14] / 1000)
        battery_cell_voltage3 = float(registers[15] / 1000)
        battery_cell_voltage4 = float(registers[16] / 1000)
        battery_cell_voltage5 = float(registers[17] / 1000)
        battery_cell_voltage6 = float(registers[18] / 1000)
        battery_cell_voltage7 = float(registers[19] / 1000)
        battery_cell_voltage8 = float(registers[20] / 1000)

        return {
            "ticks": ticks,
            "Maximum_Voltage": Maximum_Voltage,
            "Minimum_Voltage": Minimum_Voltage,
            "battery_voltage": voltage,
            "battery_design_capacity": Battery_Capacity,
            "battery_current_consumption": Current_Consumption,
            "Watt_Hour": Watt_Hour,
            "battery_percentage": Battery_Percentage,
            "battery_temperature": Temp,
            "Remaining_Run_Time": Remaining_Run_Time,
            "battery_current_capacity": Remaining_Capacity,
            "Charge_Time": Charge_Time,
            "battery_cell_voltage": float(
                battery_cell_voltage1
                + battery_cell_voltage2
                + battery_cell_voltage3
                + battery_cell_voltage4
                + battery_cell_voltage5
                + battery_cell_voltage6
                + battery_cell_voltage7
                + battery_cell_voltage8
            ),
        }
