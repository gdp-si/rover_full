"""Abstract module for sensor definition"""
from abc import ABC, abstractmethod
from typing import Any


class SensorDefinition(ABC):
    """Abstract class for sensor definition

    Args:
        ABC (Abstract Class): [description]
    """

    @abstractmethod
    def is_active(self) -> bool:
        """Checks if the sensor is active or not"""

    @abstractmethod
    def get_info(self) -> str:
        """Returns the sensor information in string format"""

    # pylint: disable=arguments-differ
    @abstractmethod
    def update(self, *args, **kwargs) -> Any:
        """Update the sensor value and returns the corresponding value"""

    @abstractmethod
    def print_msg(self):
        """Prints relevant sensor message"""
