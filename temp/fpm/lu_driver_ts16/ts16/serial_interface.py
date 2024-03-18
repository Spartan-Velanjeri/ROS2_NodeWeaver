#!/bin/python3
#
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

# flake8: noqa

"""Serial interface for Leica TS16."""

import sys

import rclpy
from rclpy.node import Node
import serial
# from leica_ts16_driver.interface import Interface


class SerialInterface():
    """Serial interface for Leica TS16."""

    def __init__(self,serial_port,serial_baud,serial_timeout):
        """Initialize interface."""
        self.serial_port = serial_port
        self.serial_baud = serial_baud
        self.serial_timeout = serial_timeout

    def connect(self):
        """Connect to device."""
        try:
            # self._con = serial.Serial(port=self.serial_port, baudrate=self.serial_baud, write_timeout=1, rtscts=0)
            self._con = serial.Serial(port=self.serial_port, baudrate=self.serial_baud, timeout=self.serial_timeout, rtscts=0)
        except serial.serialutil.SerialException:
            print("ERROR: Unable to connect to serial device {}".format(self.serial_port))
            sys.exit(0)

    def readlines(self, single=False):
        """Read line from device.

        Args:
            single: just return the next single response
        """
        try:
            if not single:
                return [self._con.readline().decode()]
            else:
                return self._con.readline().decode()
        except serial.SerialException:
            print('ERROR: Serial connection closed, exiting...')
            sys.exit(0)

    def write(self, data):
        """Write to device.

        Args:
            data: data to be written
        """
        try:
            self._con.write(data)
        except serial.SerialException:
            print('ERROR: Serial connection closed, exiting...')
            sys.exit(0)

    def reset(self):
        """Reset buffer."""
        self._con.reset_input_buffer()

    def close(self):
        """Destructor."""
        self._con.close()
