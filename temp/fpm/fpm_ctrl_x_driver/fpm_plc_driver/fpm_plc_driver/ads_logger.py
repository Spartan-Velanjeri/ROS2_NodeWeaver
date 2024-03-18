# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction
# editing, distribution, as well as in the event of applications for industrial
# property rights.

"""Ads logger module. Extension of the standard python logger to support kwargs.
Throttling argument can be used to limit the number of log messages per second."""

from logging import Logger


class AdsLogger:
    """Ads logger class. Extension of the standard python logger to support kwargs."""

    def __init__(self, name: str, level: int):

        self.logger = Logger(name, level)

    def debug(self, msg: str, **kwargs):  # pylint: disable=unused-argument
        """Log a message with severity 'DEBUG'."""
        self.logger.debug(msg)

    def info(self, msg: str, **kwargs):  # pylint: disable=unused-argument
        """Log a message with severity 'INFO'."""
        self.logger.info(msg)

    def warning(self, msg: str, **kwargs):  # pylint: disable=unused-argument
        """Log a message with severity 'WARNING'."""
        self.logger.warning(msg)

    def error(self, msg: str, **kwargs):  # pylint: disable=unused-argument
        """Log a message with severity 'ERROR'."""
        self.logger.error(msg)

    def critical(self, msg: str, **kwargs):  # pylint: disable=unused-argument
        """Log a message with severity 'CRITICAL'."""
        self.logger.critical(msg)

    def exception(self, msg: str, **kwargs):  # pylint: disable=unused-argument
        """Log a message with severity 'ERROR'."""
        self.logger.exception(msg)

    def log(
        self, level: int, msg: str, **kwargs
    ):  # pylint: disable=unused-argument
        """Log 'msg' with severity 'level'."""
        self.logger.log(level, msg)


def get_logger(name: str, level: int):
    """Get logger."""
    try:
        from rclpy.impl.rcutils_logger import RcutilsLogger

        _logger = RcutilsLogger(name)
        _logger.set_level(level)
        return _logger
    except ImportError:
        return AdsLogger(name, level)
