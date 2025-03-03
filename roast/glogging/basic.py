"""Logging module for basic configurations.
    Aimed only for development and debugging"""
import logging
import os

import coloredlogs

LEVEL = {
    "CRITICAL": logging.CRITICAL,
    "ERROR": logging.ERROR,
    "WARN": logging.WARNING,
    "INFO": logging.INFO,
    "DEBUG": logging.DEBUG,
}

LOG_LEVEL = os.environ.get("ROAST_LOGLEVEL", "INFO")


coloredFormatter = coloredlogs.ColoredFormatter(
    fmt="[%(asctime)s] [%(hostname)s] [%(name)s] [%(levelname)s] %(message)s",
    level_styles=dict(
        debug=dict(color="white"),
        info=dict(color="blue"),
        warning=dict(color="yellow", bright=True),
        error=dict(color="red", bold=True, bright=True),
        critical=dict(color="black", bold=True, background="red"),
    ),
    field_styles=dict(
        name=dict(color="white"),
        asctime=dict(color="white"),
        funcName=dict(color="white"),
        lineno=dict(color="white"),
    ),
    datefmt="%Y-%m-%d %H:%M:%S",
)


class Logger:
    """Logger for debugging and development"""

    def __init__(self, _module_name: str = "", _level=LEVEL[LOG_LEVEL]) -> None:
        self.logger = logging.getLogger(_module_name)

        if not self.logger.handlers:
            coloredlogs.install(logger=self.logger)
            console = logging.StreamHandler()
            console.setFormatter(fmt=coloredFormatter)
            console.addFilter(coloredlogs.HostNameFilter())
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
