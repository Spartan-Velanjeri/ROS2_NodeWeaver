# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction
# editing, distribution, as well as in the event of applications for industrial
# property rights.

"""Ads client for the ptu."""
import socket
import time
from typing import Any, Dict

import pyads

# ##############################################################################
# paths to data-layer variables (plc)
# ptu_rpm_setpoint - setpoint for the rpm of the ptu; left: <0, right: >0; 0: stop
#                    limit +-500 rpm
# ptu_RPM_actual - actual rpm of the ptu
# ptu_rpm_max - maximum rpm of the ptu (active only if ptu_asc is set)
# ptu_led - led control
# ptu_asc - adaptive speed control (speed is controlled by the ptu)
# ptu_sensor_state - state of the sensor (acceleration)
# ptu_sensor_value - acceleration values
# ptu_state - state of the ptu
# ptu_I_actual_M - actual current of the motor
# ptu_I_actual_A - actual current of the battery
# ptu_T_actual_M - actual temperature of the motor
# ptu_T_actual_MCU - actual temperature of the MCU
# ptu_DutyCycle - duty cycle of the MCU (speed controller)
# ##############################################################################

# if a new variable is added, it has to be added to the PTU_VARIABLES_NAMES list
PTU_RPM_SETPOINT = ("GVL_Datalayer.ptu_rpm_setpoint", pyads.PLCTYPE_INT)
PTU_RPM_ACTUAL = ("GVL_Datalayer.ptu_RPM_actual", pyads.PLCTYPE_INT)
PTU_RPM_MAX = ("GVL_Datalayer.ptu_rpm_max", pyads.PLCTYPE_INT)
PTU_LED = ("GVL_Datalayer.ptu_led", pyads.PLCTYPE_BOOL)
PTU_ASC = ("GVL_Datalayer.ptu_asc", pyads.PLCTYPE_BOOL)
PTU_SENSOR_STATE = ("GVL_Datalayer.ptu_sensor_state", pyads.PLCTYPE_WORD)
PTU_SENSOR_VALUE = ("GVL_Datalayer.ptu_sensor_value", pyads.PLCTYPE_ARR_INT(2))
PTU_STATE = ("GVL_Datalayer.ptu_state", pyads.PLCTYPE_BYTE)
PTU_I_ACTUAL_M = ("GVL_Datalayer.ptu_I_actual_M", pyads.PLCTYPE_INT)
PTU_I_ACTUAL_A = ("GVL_Datalayer.ptu_I_actual_A", pyads.PLCTYPE_INT)
PTU_T_ACTUAL_M = ("GVL_Datalayer.ptu_T_actual_M", pyads.PLCTYPE_INT)
PTU_T_ACTUAL_MCU = ("GVL_Datalayer.ptu_T_actual_MCU", pyads.PLCTYPE_INT)
PTU_DUTY_CYCLE = ("GVL_Datalayer.ptu_DutyCycle", pyads.PLCTYPE_INT)

PTU_VARIABLES_NAMES = [
    PTU_RPM_SETPOINT[0],
    PTU_RPM_ACTUAL[0],
    PTU_RPM_MAX[0],
    PTU_LED[0],
    # PTU_ASC[0],
    PTU_SENSOR_STATE[0],
    PTU_SENSOR_VALUE[0],
    PTU_STATE[0],
    PTU_I_ACTUAL_M[0],
    PTU_I_ACTUAL_A[0],
    PTU_T_ACTUAL_M[0],
    PTU_T_ACTUAL_MCU[0],
    PTU_DUTY_CYCLE[0],
]

THROTTLE_ERROR_LOGGING = 2


class AdsClient:
    """ADS client for communication with the Beckhoff PLC. Module
    functionality can be emulated by providing emulations_parameters dict.
    """

    def __init__(
        self,
        logger: Any,
        emulation: Dict = {"active": False, "ptu_state": 0},
    ):
        """Ads (plc) client.

        Args:
            logger (Any): Logger.
            emulation (Dict, optional): Defaults to {}.
                Emulation parameters (dict key-value):
                --key--         --type--    --description--
                active          bool        If true emulation is active
                                            (no communication with hw).
                ptu_state       int         Emulated state of the ptu.
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

        self.read_cycle_time = 0.1  # read cycle time in seconds
        self._last_read_time = 0.0  # last read time

        self.logger = logger
        self._emulation_parameters = emulation

        # ads communication client
        try:
            import pyads
        except FileNotFoundError as e:
            self.logger.error("ADS client library not installed. Error: %s", e)
            raise FileNotFoundError(
                "ADS client library not installed."
            ) from FileNotFoundError
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

        self._connect()

        self.ptu_variables: Dict = {}
        self.read_ptu_variables()

        self.logger.info(f"ADS client initialization done.PLC IP={self.ip_address}.")

    @property
    def emulation(self) -> bool:
        """Emulation activation.

        Returns:
            bool: Emulation status flag.
        """
        return self._emulation_parameters.get("active", False)

    @property
    def rpm(self) -> int:
        """Actual RPM of the ptu."""

        return self.get_variable(PTU_RPM_ACTUAL[0])

    @property
    def rpm_set_point(self) -> int:
        """RPM setpoint of the PTU.

        Returns:
            int: RPM setpoint.
        """
        return self.get_variable(PTU_RPM_SETPOINT[0])

    @rpm_set_point.setter
    def rpm_set_point(self, value: int):
        """Set RPM setpoint of the PTU.

        Args:
            value (int): RPM setpoint.
        """
        self._write_ads(variable=PTU_RPM_SETPOINT, value=value)

    @property
    def rpm_max(self) -> int:
        """Maximum RPM of the PTU.

        Returns:
            int: Maximum RPM.
        """
        return self.get_variable(PTU_RPM_MAX[0])

    @rpm_max.setter
    def rpm_max(self, value: int):
        """Set maximum RPM of the PTU.

        Args:
            value (int): Maximum RPM.
        """
        self._write_ads(variable=PTU_RPM_MAX, value=value)

    @property
    def led(self) -> bool:
        """LED control.

        Returns:
            bool: LED state.
        """
        return self.get_variable(PTU_LED[0])

    @led.setter
    def led(self, value: bool):
        """Set maximum led.

        Args:
            value (bool): Led value.
        """
        self._write_ads(variable=PTU_LED, value=value)

    @property
    def motor_current(self):
        """Actual current of the motor in [A]."""

        return self.get_variable(PTU_I_ACTUAL_M[0])

    @property
    def accu_current(self):
        """Actual current of the accu in [A]."""
        return self.get_variable(PTU_I_ACTUAL_A[0])

    @property
    def motor_temperature(self):
        """Actual temperature of the motor in [°C]."""

        return self.get_variable(PTU_T_ACTUAL_M[0])

    @property
    def mcu_temperature(self):
        """Actual temperature of the accu in [°C]."""
        return self.get_variable(PTU_T_ACTUAL_MCU[0])

    @property
    def ptu_data(self) -> Dict:
        """Reads all PTU data from the PLC."""
        return self.get_variable()

    def ptu_switch_on(
        self, rpm_setpoint: int, rpm_max, asc: bool, led: bool = False
    ) -> None:
        """Switching ptu on.

        Args:
            rpm_setpoint (int): Rotation setpoint in RPM.
            asc (bool): Activation of the asc function.
            rpm_max (int): Max RPM if ASC is activated.
            led (bool): Led light command.
        """

        variable_write_list = {
            PTU_RPM_MAX[0]: rpm_max,
            PTU_RPM_SETPOINT[0]: rpm_setpoint,
            PTU_ASC[0]: asc,
            PTU_LED[0]: led,
        }

        try:
            self.logger.debug(f"Switching ptu on: {variable_write_list}.")
            result = self.client.write_list_by_name(
                data_names_and_values=variable_write_list
            )
            self.logger.debug(str(result))
        except pyads.pyads_ex.ADSError as e:
            self.logger.error(f"Error in switching the ptu on. Details{e}")

    def ptu_switch_off(self):

        variable_write_list = {
            PTU_RPM_MAX[0]: 0,
            PTU_LED[0]: 0,
        }

        try:
            self.logger.debug(f"Switching ptu off: {variable_write_list}.")
            result = self.client.write_list_by_name(
                data_names_and_values=variable_write_list
            )
            self.logger.debug(str(result))
        except pyads.pyads_ex.ADSError as e:
            self.logger.error(f"Error in switching the ptu on. Details{e}")

    @property
    def connected(self) -> bool:
        """Provides status of the connection to the plc.

        Returns:
            bool: Connection status.
        """

        if self.emulation:
            return True

        return self.client.is_open

    def get_variable(self, variable_name: str = None) -> Any:
        """Reads variable from the PLC. Variables are read in cycle time
        depending on the read cycle time setting. If name of the variable
        is not specified thw whole data structure in returns as Dict."""

        # read variables only if the read cycle time has passed
        if time.time() - self._last_read_time > self.read_cycle_time:
            self.read_ptu_variables()
            self._last_read_time = time.time()

        if variable_name is None:
            return self.ptu_variables

        return self.ptu_variables[variable_name]

    def read_ptu_variables(self) -> None:
        """Reads all variables from the PLC."""
        if not self.connected:
            self._connect()

        try:
            variables = self.client.read_list_by_name(data_names=PTU_VARIABLES_NAMES)
            self.ptu_variables = variables
        except pyads.pyads_ex.ADSError as e:
            self.logger.fatal(
                f"PLC driver internal error (read_ptu_variables). Details:{e}",
                throttle_duration_sec=THROTTLE_ERROR_LOGGING,
            )
            raise IOError from e

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
                    "PLC not connected.",
                    throttle_duration_sec=THROTTLE_ERROR_LOGGING,
                )

            self.logger.error(
                f"ADS error: {e}", throttle_duration_sec=THROTTLE_ERROR_LOGGING
            )

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
