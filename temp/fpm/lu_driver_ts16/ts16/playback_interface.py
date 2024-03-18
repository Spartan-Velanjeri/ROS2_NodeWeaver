#!/bin/python3
#
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

# flake8: noqa

"""Playback interface for Leica TS16."""

import rclpy
import serial
import time
# from ts16.interface import Interface


class PlaybackInterface():
    """Playback interface for Leica TS16."""

    def __init__(self,playback_path):
        """Initialize interface."""
        self.playback_path = playback_path

    def time_sync(self, system_time):
        """Fake playback timesync.

        Args:
            system_time: system time to sync todo
        """
        with open(self.playback_path) as f:
            first_line = ''
            while not first_line.startswith('TS'):
                first_line = f.readline()

        t_edm = float(first_line.split(',')[-1]) / 1000
        time_offset = system_time - t_edm

        return time_offset

    def connect(self):
        """Connect to device."""
        self._con = open(self.playback_path, 'rb')

    def readlines(self, single=False):
        """Read line from device."""
        time.sleep(0.05)
        line = self._con.readline().decode()
        if not single:
            return [line]
        else:
            return line

    def write(self, _):
        """Write to device."""
        pass

    def reset(self):
        """Reset buffer."""
        pass

    def close(self):
        """Destructor."""
        self._con.close()
