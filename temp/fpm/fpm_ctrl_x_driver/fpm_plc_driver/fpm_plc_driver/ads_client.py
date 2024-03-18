# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction
# editing, distribution, as well as in the event of applications for industrial
# property rights.

"""Ads client for the lift."""

import socket
import threading
import time
from typing import Dict

import pyads
from rclpy.impl.rcutils_logger import RcutilsLogger

from fpm_plc_driver.drive_state import DriveState
from fpm_plc_driver.state import State

# paths to data-layer variables (plc)
DRIVE_SETPOINT = ("GVL_Datalayer.drive_setpoint", pyads.PLCTYPE_REAL)
DRIVE_POSITION = ("GVL_Datalayer.drive_position", pyads.PLCTYPE_REAL)
DRIVE_VELOCITY = ("GVL_Datalayer.drive_velocity", pyads.PLCTYPE_DINT)
DRIVE_START = ("GVL_Datalayer.drive_start", pyads.PLCTYPE_BOOL)
DRIVE_STOP = ("GVL_Datalayer.drive_stop", pyads.PLCTYPE_BOOL)
DRIVE_PROFILE_VELOCITY = (
    "GVL_Datalayer.drive_profile_velocity",
    pyads.PLCTYPE_REAL
)

DRIVE_STATE = ("GVL_Datalayer.drive_state", pyads.PLCTYPE_WORD)
DRIVE_ERROR = ("GVL_Datalayer.drive_error", pyads.PLCTYPE_WORD)
DRIVE_DISABLE = ("GVL_Datalayer.drive_disable", pyads.PLCTYPE_BOOL)
DRIVE_ENABLE = ("GVL_Datalayer.drive_enable", pyads.PLCTYPE_BOOL)
DRIVE_ERROR_RESET = ("GVL_Datalayer.drive_error_reset", pyads.PLCTYPE_BOOL)

VACUUM_TOGGLE = ("GVL_Datalayer.vacuum_toggle", pyads.PLCTYPE_BOOL)
VACUUM_STATE = ("GVL_Datalayer.vacuum_state", pyads.PLCTYPE_WORD)

THROTTLE_LIFT_POSITION_LOGGING = 10  # seconds
THROTTLE_ERROR_LOGGING = 2


class AdsClient:
    """ADS client for communication with the Beckhoff PLC. Module
    functionality can be emulated by providing emulations_parameters dict.
    """

    def __init__(self, logger: RcutilsLogger, emulation: Dict = {}):
        """Ads (plc) client.

        Args:
            logger (RcutilsLogger): ROS logger.
            emulation (Dict, optional): Defaults to {}.
                Emulation parameters (dict key-value):
                --key--         --type--    --description--
                active          bool        If true emulation is active
                                            (no communication with hw).
                drive_state     int         Emulated drive state.
        """
        # settings of the plc
        self.ip_address = "192.168.2.6"
        # self.ams_net_id = self.ip_address + ".1.1"
        self.ams_net_id = "5.137.14.247.1.1"
        self.user = "Administrator"
        self.password = "1"

        # settings of the fpm cu
        self.fpm_ip_address = "192.168.2.2"
        self.fpm_ams_net_id = self.fpm_ip_address + ".1.1"

        self.logger = logger
        # ads communication client
        # try:
        #     import pyads
        # except FileNotFoundError as e:
        #     self.logger.error("ADS client library not installed. Error: %s",e)
        #     raise FileNotFoundError ("ADS client library not installed.") from FileNotFoundError
        try:
            pyads.add_route_to_plc(
                sending_net_id=self.fpm_ams_net_id,
                adding_host_name=self.fpm_ip_address,
                ip_address=self.ip_address,
                username=self.user,
                password=self.password,
                route_name="route-to-fpm-cu",
            )
        except socket.timeout:
            # adding the route only on driver start
            # if route is already added to the plc this exception is not problem
            # there is LAN connection problem with the plc
            self.logger.error("Adding route to the PLC failed.")

        self.client = pyads.Connection(
            ams_net_id=self.ams_net_id,
            ams_net_port=851,
            ip_address=self.ip_address,
        )

        self._vacuum_state = State.ready

        self._connect()

        self._emulation_parameters = emulation

        self.logger.info(f"ADS client initialization done.PLC IP={self.ip_address}.")

    @property
    def emulation(self) -> bool:
        """Emulation activation.

        Returns:
            bool: Emulation status flag.
        """
        return self._emulation_parameters.get("active", False)

    @property
    def _drive_state(self) -> DriveState:
        """State of the motion drive.

        Raises:
            NotImplementedError: PLC part not implemented.

        Returns:
            int: Drive state (FIXME: which ones)
        """

        if self.emulation:
            # emulation is active, return emulation state
            return DriveState(self._emulation_parameters["drive_state"])

        # if not self.connected:
        #     raise NotImplementedError
        try:
            # drive_state = self.client.read_by_name(
            #     data_name=DRIVE_STATE[0], plc_datatype=DRIVE_STATE[1]
            # )
            drive_state = self._read_ads(variable=DRIVE_STATE)
        except pyads.pyads_ex.ADSError as e:
            self.logger.fatal(
                f"PLC driver internal error (_drive_state). Details:{e}",
                throttle_duration_sec=THROTTLE_ERROR_LOGGING,
            )
            raise IOError from e

        return DriveState(drive_state)

    @property
    def lift_state(self) -> State:
        """Returns state of the lift unit.

        Returns:
            State: State of the lift.
        """

        try:
            drive_state = self._drive_state
            if drive_state in (
                DriveState.en_DEFAULT,
                DriveState.en_WAIT,
                DriveState.en_Not_Ready_To_Switch_On,
                DriveState.en_Ready_to_Switch_On,
                DriveState.en_Quick_Stop_Active,
            ):
                # not_ready state
                return State.not_ready
            elif drive_state in (DriveState.en_Switched_On,):
                # ready state
                return State.ready
            elif drive_state in (
                DriveState.en_Operation_Mode_Pos,
                DriveState.en_Operation_Enable,
            ):
                # operation state
                return State.operation
            elif drive_state in (DriveState.en_Switch_On_Disabled,):
                # disabled state
                return State.disabled
            elif drive_state in (
                DriveState.en_Fault,
                DriveState.en_Fault_DS402,
                DriveState.en_Fault_Reaction_Active,
            ):
                # fault state
                return State.fault

            else:
                # unknown drive state
                self.logger.fatal(f"Unknown drive state received:{drive_state}.")
                return State.fault

        except IOError:
            return State.fault

        except BaseException as e:
            # unknown exception
            self.logger.fatal(
                f"Unknown exception:{e}.", throttle_duration_sec=THROTTLE_ERROR_LOGGING
            )
            return State.fault

    @property
    def connected(self) -> bool:
        """Provides status of the connection to the plc.

        Returns:
            bool: Connection status.
        """

        if self.emulation:
            return True

        return self.client.is_open

    @property
    def in_motion(self) -> bool:
        """Motion flag.

        Raises:
            IOError: PLC not connected.

        Returns:
            bool: True if drive in motion.
        """
        # if not self.connected and not self.emulation:
        #     raise NotImplementedError

        if self._drive_state == DriveState.en_Operation_Enable:
            return True
        else:
            return False

    @property
    def lift_position(self) -> float:
        """Position of the lift (absolute) in mm.

        Raises:
            IOError: Raised when error occurs (ADS or drive).

        Returns:
            float: Lift position.
        """

        if self.emulation:
            return self._emulation_parameters["position"]

        # if not self.connected:
        #     # this check is fast, read_sync below takes some seconds
        #     raise NotImplementedError

        try:
            # position = self.client.read_by_name(
            #     data_name=DRIVE_POSITION[0], plc_datatype=DRIVE_POSITION[1]
            # )
            position = self._read_ads(variable=DRIVE_POSITION)
        except pyads.pyads_ex.ADSError as e:
            self.logger.fatal(
                f"PLC driver internal error (lift_position). Details:{e}",
                throttle_duration_sec=THROTTLE_ERROR_LOGGING,
            )

            raise
        return position

    @property
    def lift_position_meter(self) -> float:
        "Returns lift position in meter."
        return round((self.lift_position / 1000), 3)

    @property
    def lift_velocity(self) -> int:
        """Lift velocity in mm/sec.
        Returns:
            int: velocity
        """

        if self.emulation:
            return self._emulation_parameters["velocity"]

        # if not self.connected:
        #     raise NotImplementedError

        try:
            # velocity = self.client.read_by_name(
            #     data_name=DRIVE_VELOCITY[0], plc_datatype=DRIVE_VELOCITY[1]
            # )
            velocity = self._read_ads(variable=DRIVE_VELOCITY)
        except pyads.pyads_ex.ADSError as e:
            self.logger.fatal(
                f"PLC driver internal error (lift_velocity). Details:{e}",
                throttle_duration_sec=THROTTLE_ERROR_LOGGING,
            )
            raise

        return velocity

    @property
    def lift_velocity_meter_sec(self) -> float:
        """Lift velocity in m/sec.
        Returns:
            float: Lift velocity.
        """
        return round((self.lift_velocity / 1000), 3)

    @property
    def velocity_profile(self) -> float:
        """Lift velocity profile in mm/sec.
        Returns:
            float: Lift velocity profile in mm/sec.
        """
        try:

            # velocity_profile = self.client.read_by_name(
            #     data_name=DRIVE_PROFILE_VELOCITY[0],
            #     plc_datatype=DRIVE_PROFILE_VELOCITY[1],
            # )
            velocity_profile = self._read_ads(variable=DRIVE_PROFILE_VELOCITY)
        except pyads.pyads_ex.ADSError as e:
            self.logger.fatal(
                f"PLC driver internal error (velocity_profile). Details:{e}",
                throttle_duration_sec=THROTTLE_ERROR_LOGGING,
            )
            raise

        return velocity_profile

    def enable_lift(self):
        """Enables lift (drive)."""
        raise NotImplementedError

    def disable_lift(self):
        """Disables lift (drive)."""
        raise NotImplementedError

    def stop_lift(self):
        """Stops motion of the lift."""

        # FIXME: Should stop command be sent in any state of the drive?

        # if not self.connected:
        #     # this check is fast, read_sync below takes some seconds
        #     raise NotImplementedError

        self._write_ads(variable=DRIVE_STOP, value=True)

    def reset_lift_error(self) -> None:
        """Resets drive error."""

        # if not self.connected:
        #     raise NotImplementedError

        self._write_ads(
            variable=DRIVE_ERROR_RESET,
            value=True,
        )

        self.logger.info("Drive error reset.")

    def set_velocity_profile(self, velocity_profile: int) -> None:
        """Setting velocity profile
        Args:
            velocity_profile (int): Profile velocity in mm/sec.
        Raises:
            IOError: PLC not connected.
            IOError: Variable not set.
            IOError: Variable not set.
        """
        # if not self.connected:
        #     raise NotImplementedError

        self._write_ads(
            variable=DRIVE_PROFILE_VELOCITY,
            value=velocity_profile,
        )

        self.logger.info("Drive Profile velocity set.")

    def move_lift_absolute(self, position_setpoint: int) -> None:
        """Commanding lift to move absolute.

        Args:
            position_setpoint (int): Absolute position in mm.

        """

        if self.emulation:
            self._motion_emulation_thread = threading.Thread(
                target=self._motion_emulation, args=(position_setpoint,)
            )
            self._motion_emulation_thread.start()

            time.sleep(0.2)
            if not self._motion_emulation_thread.is_alive():
                time.sleep(1)

            return

        # this check is fast, read_sync below takes some seconds
        # if not self.connected:
        #     raise NotImplementedError

        if self.lift_state not in (State.ready, State.operation):
            raise IOError(f"DRIVE NOT READY. STATE:{self.lift_state} ")

        # write position_setpoint

        self._write_ads(
            variable=DRIVE_SETPOINT,
            value=position_setpoint,
        )

        self.logger.info(
            "Move lift to absolute position" f" {position_setpoint} commanded."
        )

        # command motion
        self._write_ads(
            variable=DRIVE_START,
            value=True,
        )

        self.logger.debug("Start MOVE set to TRUE.")

        start_time = time.time()
        while self.lift_state != State.operation:
            if time.time() > start_time + 1:
                # wait time passed, state not operational raise error
                raise IOError("LIFT MOVE FAILED")

    def start_vacuum(self) -> None:
        """Starts vacuum unit.

        Raises:
            IOError: Error in starting.
        """

        if self.emulation:
            self._vacuum_state = State.operation
            self.logger.info("EMULATION: Vacuum unit switched on.")
            return

        # if not self.connected:
        #     raise NotImplementedError

        self._write_ads(
            variable=VACUUM_TOGGLE,
            value=True,
        )

        self._vacuum_state = State.operation
        self.logger.info("Vacuum unit switched on.")

    def stop_vacuum(self) -> None:
        """Stops vacuum unit.

        Raises:
            IOError: Error in stopping unit.
        """

        if self.emulation:
            self._vacuum_state = State.ready
            self.logger.info("EMULATION: Vacuum unit switched off.")
            return

        # if not self.connected:
        #     # this check is fast, read_sync below takes some seconds
        #     raise NotImplementedError

        self._write_ads(
            variable=VACUUM_TOGGLE,
            value=True,
        )

        self._vacuum_state = State.ready
        self.logger.info("Vacuum unit switched off.")

    @property
    def vacuum_state(self) -> State:
        """State of the vacuum unit.

        Returns:
            State: Vacuum unit state.
        """
        return self._vacuum_state

    def _connect(self):
        """Initialization of the client for communication with the
        ADS.
        """
        try:
            self.client.open()
        except pyads.pyads_ex.ADSError as e:
            if e.err_code == 6:
                self.logger.error(
                    "ADS server not started.",
                    throttle_duration_sec=THROTTLE_ERROR_LOGGING,
                )
            elif e.err_code == 7:
                self.logger.error(
                    "PLC not connected.", throttle_duration_sec=THROTTLE_ERROR_LOGGING
                )

            self.logger.error(
                f"ADS error: {e}", throttle_duration_sec=THROTTLE_ERROR_LOGGING
            )

    def _motion_emulation(self, position_setpoint: int):
        """Emulation of drive position motion.

        Args:
            position_setpoint (int): New position reference.
        """
        print("Called emulation.")
        TIME_STEP = 0.5
        velocity_profile = self._emulation_parameters["velocity_profile"]
        position_increment = velocity_profile * TIME_STEP
        self._emulation_parameters["drive_state"] = 7  # needed here for tests
        position_reached = False

        while not position_reached:
            time.sleep(TIME_STEP)
            position = self._emulation_parameters["position"]
            delta = position - position_setpoint

            if abs(delta) <= position_increment:
                # reached
                position_reached = True
                self._emulation_parameters["position"] = position_setpoint
                self._emulation_parameters["velocity"] = 0
                self._emulation_parameters["drive_state"] = 5

            elif delta < 0:
                # move up
                self._emulation_parameters["position"] += position_increment
                self._emulation_parameters["velocity"] = self._emulation_parameters[
                    "velocity_profile"
                ]
                self._emulation_parameters["drive_state"] = 7
            else:
                # move down
                self._emulation_parameters["position"] -= position_increment
                self._emulation_parameters["velocity"] = self._emulation_parameters[
                    "velocity_profile"
                ]
                self._emulation_parameters["drive_state"] = 7

    def _read_ads(self, variable):
        """Reads ads variable."""

        if not self.connected:
            self._connect()

        content = self.client.read_by_name(
            data_name=variable[0], plc_datatype=variable[1]
        )

        return content

    def _write_ads(self, variable, value):
        """Writes plc variable."""

        if not self.connected:
            self._connect()

        self.client.write_by_name(
            data_name=variable[0],
            value=value,
            plc_datatype=variable[1],
        )


if __name__ == "__main__":
    import logging

    client = AdsClient(logger=logging.getLogger(__name__))

    # Testing vacuum
    print("--------------Vacuum test---------------")
    print(f"Vacuum state: {client.vacuum_state}")
    print("Switching vacuum on...")
    client.start_vacuum()
    print(f"Vacuum state: {client.vacuum_state}")
    print("Switching vacuum off...")
    client.stop_vacuum()
    print(f"Vacuum state: {client.vacuum_state}")

    print("--------------Drive test---------------")
    # read data
    print(f"Emulation: {client.emulation}")
    print(f"Drive state: {client._drive_state}")
    print(type(client._drive_state))
    print(f"Lift state: {client.lift_state}")
    print(f"Connected: {client.connected}")
    print(f"In motion: {client.in_motion}")
    print(f"Position: {client.lift_position} [mm]")
    print(f"Position: {client.lift_position_meter} [m]")
    print(f"Velocity: {client.lift_velocity} [mm/sec]")
    print(f"Velocity: {client.lift_velocity_meter_sec} [mm/sec]")
    print(f"Velocity profile: {client.velocity_profile} [mm/sec]")

    print(f"Velocity profile: {client.velocity_profile} [mm/sec]")
    print("Write velocity profile")
    client.set_velocity_profile(velocity_profile=50)
    print(f"Velocity profile: {client.velocity_profile} [mm/sec]")

    print("Moving lift to position 0")
    client.move_lift_absolute(position_setpoint=0)
    while client.in_motion:
        time.sleep(0.5)
    print("Moving lift to position 100")
    client.move_lift_absolute(position_setpoint=100)
    while client.in_motion:
        time.sleep(0.5)
    print("Moving lift to position 0")
    client.move_lift_absolute(position_setpoint=0)
    while client.in_motion:
        time.sleep(0.5)
