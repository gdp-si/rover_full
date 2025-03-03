"""Logging module for basic configurations.
    File handling and saving the log to text"""
import logging
import os
from datetime import datetime

from roast.settings.sys import LOG_PATH as DEFAULT_LOG_PATH

LEVEL = {
    "CRITICAL": logging.CRITICAL,
    "ERROR": logging.ERROR,
    "WARN": logging.WARNING,
    "INFO": logging.INFO,
    "DEBUG": logging.DEBUG,
}

LOG_LEVEL = os.environ.get("ROAST_LOGLEVEL", "INFO")
LOG_DIR = os.environ.get("ROS_LOG_DIR", DEFAULT_LOG_PATH)


class Logger:
    """Logger for debugging and development"""

    def __init__(self, _module_name: str = "", _level=LEVEL[LOG_LEVEL]) -> None:
        self.logger = logging.getLogger(_module_name)

        # Two set of configurations for logging to group the logs
        LOG_PATH = (
            os.path.abspath(os.path.join(LOG_DIR, ".."))
            if LOG_DIR is not DEFAULT_LOG_PATH
            else LOG_DIR
        )

        if not os.path.exists(LOG_PATH):
            os.makedirs(LOG_PATH)

        if not self.logger.handlers:
            self._file_name = datetime.now().strftime("%d-%m-%Y")
            console = logging.FileHandler(
                os.path.join(LOG_PATH, f"{self._file_name}-pymodule.log")
            )
            formatter = logging.Formatter(
                "[%(asctime)s] [%(name)s] [%(levelname)s] [%(message)s]"
            )
            console.setFormatter(fmt=formatter)
            self.logger.addHandler(console)
            self.logger.setLevel(_level)
            self.logger.propagate = False

    def INFO(self, _message: str) -> None:
        """Information that need to be logged

        Args:
            _message (str): Message to be logged
        """
        self.logger.info(_message)

    def DEBUG(self, _message: str) -> None:
        """Debugging messages that need to be logged

        Args:
            _message (str): Message to be logged
        """
        self.logger.debug(_message)

    def WARNING(self, _message: str) -> None:
        """Warning messages that need to be logged

        Args:
            _message (str): Message to be logged
        """
        self.logger.warning(_message)

    def ERROR(self, _message: str, traceback: bool = False) -> None:
        """Error messages that need to be logged

        Args:
            _message (str): Message to be logged
        """
        self.logger.error(_message, exc_info=traceback)

    def CRITICAL(self, _message: str, traceback: bool = False) -> None:
        """Critical error messages that need to be logged

        Args:
            _message (str): Message to be logged
        """
        self.logger.critical(_message, exc_info=traceback)
