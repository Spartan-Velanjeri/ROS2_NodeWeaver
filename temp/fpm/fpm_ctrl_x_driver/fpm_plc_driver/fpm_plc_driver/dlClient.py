# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.
"""Data layer client for communication with the Rexroth ctrlX PLC.
   The ctrl_x PC was used in FUS1 and it was replaced by Beckhoff PLC.
"""
import faulthandler
import sys
import threading
import time
from typing import Dict

import ctrlxdatalayer
from ctrlxdatalayer.variant import Result
from fpm_plc_driver.drive_state import DriveState
from fpm_plc_driver.state import State
from rclpy.impl.rcutils_logger import RcutilsLogger

faulthandler.enable()


# paths to data-layer variables (plc)
DRIVE_SETPOINT = "plc/app/Application/sym/GVL_Datalayer/drive_setpoint"
DRIVE_POSITION = "plc/app/Application/sym/GVL_Datalayer/drive_position"
DRIVE_VELOCITY = "plc/app/Application/sym/GVL_Datalayer/drive_velocity"
DRIVE_START = "plc/app/Application/sym/GVL_Datalayer/drive_start"
DRIVE_STOP = "plc/app/Application/sym/GVL_Datalayer/drive_stop"
DRIVE_PROFILE_VELOCITY = "plc/app/Application/sym/GVL_Datalayer/drive_profile_velocity"

DRIVE_STATE = "plc/app/Application/sym/GVL_Datalayer/drive_state"
DRIVE_ERROR = "plc/app/Application/sym/GVL_Datalayer/drive_error"
DRIVE_DISABLE = "plc/app/Application/sym/GVL_Datalayer/drive_disable"
DRIVE_ENABLE = "plc/app/Application/sym/GVL_Datalayer/drive_enable"
DRIVE_ERROR_RESET = "plc/app/Application/sym/GVL_Datalayer/drive_error_reset"

VACUUM_TOGGLE = "plc/app/Application/sym/GVL_Datalayer/vacuum_toggle"
VACUUM_STATE = "plc/app/Application/sym/GVL_Datalayer/vacuum_state"

THROTTLE_LIFT_POSITION_LOGGING = 10  # seconds


class DlClient:
    """Data layer client for communication with the ctrlX PLC. Module
    functionality can be emulated by providing emulations_parameters dict.
    """

    def __init__(self, logger: RcutilsLogger, emulation: Dict = {}):
        """Data layer (plc) client.

        Args:
            logger (RcutilsLogger): ROS logger.
            emulation (Dict, optional): Defaults to {}.
                Emulation parameters (dict key-value):
                --key--         --type--    --description--
                active          bool        If true emulation is active
                                            (no communication with hw).
                drive_state     int         Emulated drive state.
        """
        self.ip = "192.168.2.6"
        self.user = "boschrexroth"
        self.password = "100_Bautiro_100"
        self.datalayer_system = None

        self.logger = logger

        self._vacuum_state = State.ready

        self._init_client()

        self._emulation_parameters = emulation

        self.logger.info(f"Ctrl x client initialization done.PLC IP={self.ip}.")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        # Clean up client which is mapped internally to C++ objects
        del self.client
        del self.datalayer_system
        self.logger.info("Cleaned up client.")
        sys.stdout.flush()

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

        # FIXME: return drive_state information (plc)
        if not self.connected:
            raise IOError(Result.CLIENT_NOT_CONNECTED.name)

        result, variant = self.client.read_sync(DRIVE_STATE)

        if result is not Result.OK:
            # error in reading the value
            self.logger.debug(
                f"Error in reading the lift_position. Error={result}.",
                throttle_duration_sec=THROTTLE_LIFT_POSITION_LOGGING,
            )
            raise IOError(result.name)

        # read fine
        drive_state = DriveState(variant.get_uint16())
        return drive_state

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
            self.logger.fatal(f"Unknown exception:{e}.")
            return State.fault

    @property
    def connected(self) -> bool:
        """Provides status of the connection to the plc.

        Returns:
            bool: Connection status.
        """

        if self.emulation:
            return True

        return self.client.is_connected()

    @property
    def in_motion(self) -> bool:
        """Motion flag.

        Raises:
            IOError: PLC not connected.

        Returns:
            bool: True if drive in motion.
        """
        if not self.connected and not self.emulation:
            # this check is fast, read_sync below takes some seconds
            raise IOError(Result.CLIENT_NOT_CONNECTED.name)

        if self._drive_state == DriveState.en_Operation_Enable:
            return True
        else:
            return False

    @property
    def lift_position(self) -> float:
        """Position of the lift (absolute) in mm.

        Raises:
            IOError: Raised when error occurs (ctrlx or drive).

        Returns:
            float: Lift position.
        """

        if self.emulation:
            return self._emulation_parameters["position"]

        if not self.connected:
            # this check is fast, read_sync below takes some seconds
            raise IOError(Result.CLIENT_NOT_CONNECTED.name)

        result, variant = self.client.read_sync(DRIVE_POSITION)

        if result is not Result.OK:
            # error in reading the value
            self.logger.debug(
                f"Error in reading the lift_position. Error={result}.",
                throttle_duration_sec=THROTTLE_LIFT_POSITION_LOGGING,
            )
            raise IOError(result.name)

        # read fine
        return variant.get_float32()

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

        if not self.connected:
            raise IOError(Result.CLIENT_NOT_CONNECTED.name)
        result, variant = self.client.read_sync(DRIVE_VELOCITY)
        if result is not Result.OK:
            self.logger.debug(
                f"Error in reading the lift_velocity. Error={result}.",
                throttle_duration_sec=THROTTLE_LIFT_POSITION_LOGGING,
            )
            raise IOError(result.name)
        return variant.get_int32()

    @property
    def lift_velocity_meter_sec(self) -> float:
        """Lift velocity in m/sec.
        Returns:
            float: Lift velocity.
        """
        return round((self.lift_velocity / 1000), 3)

    def enable_lift(self):
        """Enables lift (drive)."""
        raise NotImplementedError

    def disable_lift(self):
        """Disables lift (drive)."""
        raise NotImplementedError

    def stop_lift(self):
        """Stops motion of the lift."""

        # FIXME: Should stop command be sent in any state of the drive?

        if not self.connected:
            # this check is fast, read_sync below takes some seconds
            raise IOError(Result.CLIENT_NOT_CONNECTED.name)

        with ctrlxdatalayer.variant.Variant() as out_data:
            result = out_data.set_bool8(True)
            if result is not Result.OK:
                raise IOError(result.name)
            result, _variant = self.client.write_sync(
                DRIVE_STOP,
                out_data,
            )
            if result is not Result.OK:
                raise IOError(result.name)

    def reset_lift_error(self) -> None:
        """Resets error."""

        if not self.connected:
            raise IOError(Result.CLIENT_NOT_CONNECTED.name)

        with ctrlxdatalayer.variant.Variant() as out_data:
            result = out_data.set_bool8(True)
            if result is not Result.OK:
                raise IOError(result.name)

            result, _variant = self.client.write_sync(
                DRIVE_ERROR_RESET,
                out_data,
            )

            if result is not Result.OK:
                # command failed
                raise IOError(result.name)

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
        if not self.connected:
            raise IOError(Result.CLIENT_NOT_CONNECTED.name)
        with ctrlxdatalayer.variant.Variant() as out_data:
            result = out_data.set_uint32(velocity_profile)
            if result is not Result.OK:
                raise IOError(result.name)
            result, _variant = self.client.write_sync(
                DRIVE_PROFILE_VELOCITY,
                out_data,
            )
            if result is not Result.OK:
                raise IOError(result.name)
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
        if not self.connected:
            raise IOError(Result.CLIENT_NOT_CONNECTED.name)

        if self.lift_state not in (State.ready, State.operation):
            # FIXME: Check is operation OK
            raise IOError("DRIVE NOT READY")

        # write position_setpoint
        with ctrlxdatalayer.variant.Variant() as out_data:
            result = out_data.set_float32(float(position_setpoint))
            if result is not Result.OK:
                raise IOError(result.name)
            result, _variant = self.client.write_sync(
                DRIVE_SETPOINT,
                out_data,
            )

            if result is not Result.OK:
                raise IOError(result.name)

            self.logger.info(
                "Move lift to absolute position" f" {position_setpoint} commanded."
            )

        with ctrlxdatalayer.variant.Variant() as out_data:
            result = out_data.set_bool8(True)
            if result is not Result.OK:
                raise IOError(result.name)

            result, _variant = self.client.write_sync(
                DRIVE_START,
                out_data,
            )

            if result is not Result.OK:
                # command failed
                raise IOError(result.name)

            self.logger.debug(f"Start MOVE set to TRUE:{result}")

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

        if not self.connected:
            # this check is fast, read_sync below takes some seconds
            raise IOError(Result.CLIENT_NOT_CONNECTED.name)

        with ctrlxdatalayer.variant.Variant() as out_data:
            result = out_data.set_bool8(True)

            if result.value:
                raise IOError(result.name)

            result, _variant = self.client.write_sync(VACUUM_TOGGLE, out_data)

            if result.value:
                raise IOError(result.name)

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

        if not self.connected:
            # this check is fast, read_sync below takes some seconds
            raise IOError(Result.CLIENT_NOT_CONNECTED.name)

        with ctrlxdatalayer.variant.Variant() as out_data:
            result = out_data.set_bool8(False)

            if result.value:
                raise IOError(result.name)

            result, _variant = self.client.write_sync(VACUUM_TOGGLE, out_data)
            if result:
                raise IOError(result.name)

            self._vacuum_state = State.ready
            self.logger.info("Vacuum unit switched off.")

    @property
    def vacuum_state(self) -> State:
        """State of the vacuum unit.

        Returns:
            State: Vacuum unit state.
        """
        return self._vacuum_state

    def _init_client(self):
        """Initialization of the client for communication with the
        ctrlx datalayer.
        """

        self.datalayer_system = ctrlxdatalayer.system.System("")
        self.datalayer_system.start(False)

        connection = (
            "tcp://" + self.user + ":" + self.password + "@" + self.ip + ":2069"
        )
        self.logger.debug("Ctrl x client initialization.")
        self.client = self.datalayer_system.factory().create_client(connection)

        self.logger.debug("Set client timeout.")
        result = self.client.set_timeout(
            ctrlxdatalayer.client.TimeoutSetting.PING, 5000
        )

        self.logger.debug(f"Timeout set result:{result}")

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
