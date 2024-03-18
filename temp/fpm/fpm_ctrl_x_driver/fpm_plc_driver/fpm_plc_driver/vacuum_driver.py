# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.
"""Driver for the vacuum unit.
"""

import traceback
from typing import Dict

import rclpy
from bautiro_ros_interfaces.msg import ResponseCode
from bautiro_ros_interfaces.srv import VacuumStart, VacuumStop
from fpm_plc_driver.dlClient import DlClient
from rclpy.node import Node

NODE_NAME = "vacuum_driver"


class VacuumDriver(Node):
    """Driver for the vacuum unit."""

    def __init__(self) -> None:
        """Vacuum unit driver."""
        super().__init__(NODE_NAME)

        self.plc = DlClient(
            logger=self.get_logger(), emulation=self.emulation_parameters
        )
        self.start_service = self.create_service(
            VacuumStart,
            "start_vacuum",
            self.start_vacuum_callback,
        )
        self.stop_service = self.create_service(
            VacuumStop,
            "stop_vacuum",
            self.stop_vacuum_callback,
        )

        if self.emulation:
            self.get_logger().warning("EMULATION/SIMULATION: active for vacuum driver.")

        self.get_logger().info(f"Vacuum node parameters:{self.parameters} ")

    @property
    def emulation_parameters(self) -> Dict:
        """Returns emulation parameters (if any).

        Returns:
            Dict: Emulation parameters.
        """

        emulation_param = {}
        for key, value in self.get_parameters_by_prefix(prefix="emulation").items():
            emulation_param.update(
                {key: self.get_parameter_or(name=key, alternative_value=None).value}
            )

        return emulation_param

    @property
    def emulation(self) -> bool:
        """Returns emulation flag (Ros node parameter use_sim_time.").

        Returns:
            bool: Emulation flag.
        """
        return self.get_parameter(name="use_sim_time").value

    @property
    def parameters(self) -> Dict:
        """Returns parameters (without emulation parameters.)

        Returns:
            Dict: Parameters.
        """
        parameters = {}

        for parameter in self.get_parameters_by_prefix(prefix="").keys():
            parameters.update({parameter: self.get_parameter(name=parameter).value})

        return parameters

    def start_vacuum_callback(
        self, request: VacuumStart.Request, response: VacuumStart.Response
    ) -> VacuumStart.Response:
        """Starts vacuum unit.

        Args:
            response (VacuumStart): Response data structure.

        Returns:
            VacuumStartR.Response: Response data (response code and error
                                   details if any).
        """

        error_detail = ""
        response_code = ResponseCode.UNKNOWN
        log_message = ""

        try:
            self.plc.start_vacuum()
            log_message = "Starting vacuum successful."
            response_code = ResponseCode.OKAY
        except IOError as e:
            log_message = "Starting vacuum failed."
            response_code = ResponseCode.FPM_COMMAND_FAILED
            error_detail = str(e)
        except BaseException as e:
            log_message = "Starting vacuum failed."
            response_code = ResponseCode.FPM_INTERNAL_NODE_ERROR
            error_detail = str(e)

        self._publish_log_message(
            log_message=log_message,
            response_code=response_code,
            error_detail=error_detail,
        )
        response.response_code.data = response_code
        response.error_detail = error_detail
        return response

    def stop_vacuum_callback(
        self, request: VacuumStop.Request, response: VacuumStop
    ) -> VacuumStop:
        """Stops vacuum unit.

        Args:
            response (VacuumStop): Response data structure.

        Returns:
            VacuumStop: Response data (response code and error details if any).
        """

        error_detail = ""
        response_code = ResponseCode.UNKNOWN
        log_message = ""

        try:
            self.plc.stop_vacuum()
            log_message = "Stopping vacuum successful."
            response_code = ResponseCode.OKAY
        except IOError as e:
            log_message = "Stopping vacuum failed."
            response_code = ResponseCode.FPM_COMMAND_FAILED
            error_detail = str(e)

        except BaseException as e:
            # unknown exception
            log_message = "Stopping vacuum failed."
            response_code = ResponseCode.FPM_INTERNAL_NODE_ERROR
            error_detail = str(e)

        self._publish_log_message(
            log_message=log_message,
            response_code=response_code,
            error_detail=error_detail,
        )

        response.response_code.data = response_code
        response.error_detail = error_detail
        return response

    def _publish_log_message(
        self, log_message: str, response_code: int, error_detail: str
    ):
        """Publisher logger message.

        Args:
            log_message (str): Content of the message.
            response_code (int): Response code.
            error_detail (str): Error details.
        """
        message = (
            f"{log_message} Response code: {response_code}.Error details:"
            f" {error_detail}."
        )

        if response_code:
            self.get_logger().error(message=message)
        else:
            self.get_logger().info(message=message)


def main():
    rclpy.init()

    try:
        vacuum_driver = VacuumDriver()
        try:
            rclpy.spin(vacuum_driver)
        except KeyboardInterrupt:
            vacuum_driver.get_logger().fatal("Vacuum driver keyboard interrupt.")

        except BaseException as e:
            Node(node_name=NODE_NAME).get_logger().fatal(
                f"Vacuum driver. Unknown exception:{e}."
            )

        rclpy.shutdown()

    except Exception as e:
        Node(node_name=NODE_NAME).get_logger().fatal(
            "Vacuum driver unknown exception:"
            f" {''.join(traceback.format_exception(etype=type(e), value=e, tb=e.__traceback__))}."
        )
