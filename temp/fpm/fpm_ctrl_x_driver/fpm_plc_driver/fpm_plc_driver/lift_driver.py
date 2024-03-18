# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.
"""Driver for the lift unit.
"""

import time
import traceback

import rclpy
from bautiro_ros_interfaces.action import MoveLiftAbsolute
from bautiro_ros_interfaces.msg import ResponseCode, State, StateCode
from bautiro_ros_interfaces.srv import (
    GetLiftPosition,
    MoveLiftAbsoluteStop,
    SetLiftVelocityProfile,
)
from fpm_plc_driver.ads_client import AdsClient
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster

NODE_NAME = "lift_driver"
LIFT_POS_PERIOD = 0.1  # period for publishing "lift_position" topic
THROTTLE_LIFT_POS_LOGGING = 10  # seconds
THROTTLE_ERROR_LOGGING = 2

SEGMENTS = [
    ("lift_segment1", "lift_segment2"),
    ("lift_segment2", "lift_segment3"),
    ("lift_segment3", "lift_segment4"),
]


class LiftDriver(Node):
    """Driver for the lift unit."""

    def __init__(self) -> None:
        super().__init__(NODE_NAME)

        self._state = StateCode.NOT_READY

        # flag will be set if drive stop is called
        # cleared when new motion is started
        self._stopped = False
        if self.emulation:
            self.get_logger().warning("EMULATION/SIMULATION: active for lift driver.")
            emulation_params = {
                "active": True,
                "drive_state": 5,
                "position": 100.0,  # position in mm
                "velocity": 0.0,  # velocity in mm/sec
                "velocity_profile": 80.0,  # velocity profile in mm/sec
            }
        else:
            emulation_params = {}

        self.plc = AdsClient(logger=self.get_logger(), emulation=emulation_params)

        # PARAMETERS
        self.declare_parameter(
            name="lift_position_min",
            value=0.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Minimal position that can be commanded to the lift.",
            ),
        )
        self.declare_parameter(
            name="lift_position_max",
            value=5.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Maximal position that can be commanded to the lift.",
            ),
        )
        self.declare_parameter(
            name="lift_profile_velocity_min",
            value=0.01,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Minimal velocity profile that can be commanded to the lift.",
            ),
        )
        self.declare_parameter(
            name="lift_profile_velocity_max",
            value=0.12,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Maximal velocity profile that can be commanded to the lift.",
            ),
        )
        # PUBLISHER - "lift_position"
        self.topic_lift_position = self.create_publisher(
            msg_type=Float32, topic="lift_position", qos_profile=2
        )
        self.timer_lift_pos = self.create_timer(
            timer_period_sec=LIFT_POS_PERIOD,
            callback=self.lift_position_callback,
        )

        # PUBLISHER - state
        self.topic_state = self.create_publisher(
            msg_type=State,
            topic="lift_state",
            qos_profile=QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        self.timer_topic_state = self.create_timer(
            timer_period_sec=0.1, callback=self.topic_state_callback
        )

        # ACTION SERVER move_lift_absolute
        self._action_server = ActionServer(
            self,
            action_type=MoveLiftAbsolute,
            action_name="move_lift_absolute",
            execute_callback=self.move_lift_callback,
        )

        # SERVERS
        self.create_service(
            srv_type=MoveLiftAbsoluteStop,
            srv_name="move_lift_absolute_stop",
            callback=self.stop_lift_callback,
        )

        self.create_service(
            srv_type=Empty,
            srv_name="lift_error_quit",
            callback=self.reset_drive_error,
        )

        self.create_service(
            srv_type=SetLiftVelocityProfile,
            srv_name="lift_velocity_set",
            callback=self.set_lift_velocity,
        )
        self.create_service(
            srv_type=GetLiftPosition,
            srv_name="get_lift_position",
            callback=self.get_lift_position,
        )
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.position_subs = self.create_subscription(
            Float32, "lift_position", self.tf_broadcaster_callback, 1
        )

        # state of the lift
        self._state = None

        self.get_logger().info("Initialization done.")

    def lift_position_callback(self) -> None:
        """Callback for publishing lift_position topic."""
        msg = Float32()

        try:
            lift_position = self.plc.lift_position_meter
            msg.data = lift_position

        except IOError as e:
            self.get_logger().error(
                f"Connection error PLC:{e}.",
                throttle_duration_sec=THROTTLE_LIFT_POS_LOGGING,
            )
            return

        except IOError:
            # connection problem, do not log
            pass
        except Exception as e:
            self.get_logger().error(
                f"Unknown exception in lift_position:{e}.",
                throttle_duration_sec=THROTTLE_ERROR_LOGGING,
            )
            msg.data = -1.0

        self.topic_lift_position.publish(msg)
        self.get_logger().info(
            f"Publishing lift position:{msg.data}.",
            throttle_duration_sec=THROTTLE_LIFT_POS_LOGGING,
        )

    def topic_state_callback(self) -> None:
        """Callback for publishing lift state."""

        lift_state = self.plc.lift_state

        if lift_state != self._state:
            # state of the lift has changed
            msg = State()
            msg.state.data = lift_state.value
            msg.error_detail = ""
            self.get_logger().info(
                f"Lift state changed from {self._state} to {lift_state}."
            )
            self._state = lift_state
            self.topic_state.publish(msg)

    def move_lift_callback(
        self, goal_handle: ServerGoalHandle
    ) -> MoveLiftAbsolute.Result:
        """Moving lift to requested position.

        Args:
            goal_handle (ServerGoalHandle): handle structure.

        Returns:
            MoveLiftAbsolute.Result: Result of the action.
        """

        progress_normalized = 0.0  # initialized to 0
        lift_position = -1.0  # initialized to -1 (invalid)
        self._stopped = False

        try:
            lift_setpoint = goal_handle.request.requested_target_lift_level
            lift_position = self.plc.lift_position_meter
            lift_position_min = self.get_parameter("lift_position_min").value
            lift_position_max = self.get_parameter("lift_position_max").value
            if (lift_setpoint < lift_position_min) or (
                round(lift_setpoint, 3) > lift_position_max
            ):
                raise AttributeError(
                    "Position setpoint out of range."
                    f" Min:{lift_position_min}.Max:{lift_position_max}"
                )
            starting_difference = abs(lift_position - lift_setpoint)

            feedback_msg = MoveLiftAbsolute.Feedback()

            self.plc.move_lift_absolute(position_setpoint=int(1000 * lift_setpoint))

            while self.plc.in_motion:
                lift_position = self.plc.lift_position_meter
                current_difference = abs(lift_position - lift_setpoint)

                if starting_difference:
                    progress_normalized = (
                        starting_difference - current_difference
                    ) / starting_difference
                else:
                    progress_normalized = 100.0

                # publish progress
                feedback_msg.progress_normalized = progress_normalized
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(1)

            lift_position = self.plc.lift_position_meter
            if self._stopped:
                goal_handle.abort()
            else:
                goal_handle.succeed()
            response_code = ResponseCode.OKAY
        except IOError as e:
            response_code = ResponseCode.FPM_LIFT_DRIVE_FAULT
            error_detail = str(e)

            goal_handle.abort()
            message = (
                "Move lift error. Response code:"
                f"{response_code}. Error: {error_detail} ."
            )
            self.get_logger().error(message=message)
            result = MoveLiftAbsolute.Result()
            result.final_lift_level = lift_position
            result.response_code = response_code
            return result

        except AttributeError as e:
            response_code = ResponseCode.FPM_COMMAND_NOT_ALLOWED
            error_detail = str(e)
            goal_handle.abort()
            message = (
                "Move lift error. Response code:"
                f"{response_code}. Error: {error_detail} ."
            )
            self.get_logger().error(message=message)
            result = MoveLiftAbsolute.Result()
            result.final_lift_level = lift_position
            result.response_code = response_code
            return result
        result = MoveLiftAbsolute.Result()
        result.final_lift_level = lift_position
        result.response_code = response_code
        if self._stopped:
            message = f"Move lift aborted. Response code: {response_code}.Error: ''."
        else:
            message = f"Position reached. Response code: {response_code}.Error: ''."
        self.get_logger().info(message=message)

        return result

    def stop_lift_callback(
        self,
        request: MoveLiftAbsoluteStop.Request,
        response: MoveLiftAbsoluteStop.Response,
    ) -> MoveLiftAbsoluteStop.Response:
        """Stopping lift motion.

        Args:
            response (MoveLiftAbsoluteStop.Response): Response on

        Returns:
            MoveLiftAbsoluteStop.Response: Response
        """
        log_message = ""
        response_code = ResponseCode.UNKNOWN
        error_detail = ""

        try:
            self.plc.stop_lift()
            response_code = ResponseCode.OKAY
            log_message = "Lift stopped."
            message = (
                f"{log_message} Response code: {response_code}.Error details:"
                f" {error_detail}."
            )
            self.get_logger().info(message=message)
        except IOError as e:
            response_code = ResponseCode.FPM_LIFT_DRIVE_FAULT
            log_message = "Lift stopping failed."
            error_detail = str(e)
            message = (
                f"{log_message} Response code: {response_code}.Error details:"
                f" {error_detail}."
            )
            self.get_logger().error(message=message)
        except BaseException as e:
            # unknown exception
            log_message = "Lift stopping failed."
            response_code = ResponseCode.FPM_INTERNAL_NODE_ERROR
            error_detail = str(e)
            message = (
                f"{log_message} Response code: {response_code}.Error details:"
                f" {error_detail}."
            )
            self.get_logger().fatal(message=message)

        response.response_code.data = response_code
        self._stopped = True
        return response

    def reset_drive_error(
        self,
        request: Empty.Request,
        response: Empty.Response,
    ) -> Empty.Response:
        self.get_logger().info("Drive reset called.")

        try:
            self.plc.reset_lift_error()
            return response
        except BaseException as e:
            self.get_logger().error(f"Error quitting failed. Error: {e}.")
            return response

    def set_lift_velocity(
        self,
        request: SetLiftVelocityProfile.Request,
        response: SetLiftVelocityProfile.Response,
    ) -> SetLiftVelocityProfile.Response:
        """Setting lift velocity profile.
        Args:
            response: Response of server.
        Returns:
            SetLiftVelocityProfile.Response: Response
        """
        log_message = ""
        response_code = ResponseCode.UNKNOWN
        error_detail = ""
        lift_profile_velocity_min = self.get_parameter(
            "lift_profile_velocity_min"
        ).value
        lift_profile_velocity_max = self.get_parameter(
            "lift_profile_velocity_max"
        ).value
        try:
            lift_profile_velocity_mm_sec = int(
                round(request.lift_profile_velocity, 3) * 1000
            )
            if (
                lift_profile_velocity_mm_sec < int(lift_profile_velocity_min * 1000)
            ) or (lift_profile_velocity_mm_sec > int(lift_profile_velocity_max * 1000)):
                raise AttributeError(
                    "Velocity out of range."
                    f" Min:{lift_profile_velocity_min}.Max:{lift_profile_velocity_max}"
                )
            self.plc.set_velocity_profile(velocity_profile=lift_profile_velocity_mm_sec)
            response_code = ResponseCode.OKAY
            log_message = (
                "Lift velocity profile set to"
                f" {request.lift_profile_velocity} m/sec."
            )
            message = (
                f"{log_message} Response code: {response_code}.Error details:"
                f" {error_detail}."
            )
            self.get_logger().info(message=message)
        except IOError as e:
            response_code = ResponseCode.FPM_LIFT_DRIVE_FAULT
            log_message = "Lift velocity profile change failed."
            error_detail = str(e)
            message = (
                f"{log_message} Response code: {response_code}.Error details:"
                f" {error_detail}."
            )
            self.get_logger().error(message=message)
        except AttributeError as e:
            response_code = ResponseCode.FPM_COMMAND_NOT_ALLOWED
            log_message = "Lift velocity profile change failed."
            error_detail = str(e)
            message = (
                f"{log_message} Response code: {response_code}.Error details:"
                f" {error_detail}."
            )
            self.get_logger().error(message=message)
        except BaseException as e:
            log_message = "Lift velocity profile change failed."
            response_code = ResponseCode.FPM_INTERNAL_NODE_ERROR
            error_detail = str(e)
            message = (
                f"{log_message} Response code: {response_code}.Error details:"
                f" {error_detail}."
            )
            self.get_logger().fatal(message=message)
        response.response_code.data = response_code
        response.error_detail = error_detail
        self._stopped = True
        return response

    def get_lift_position(
        self,
        request: GetLiftPosition.Request,
        response: GetLiftPosition.Response,
    ) -> GetLiftPosition.Response:
        """Returns lift position.

        Args:
            request (GetLiftPosition.Request): Empty.
            response (GetLiftPosition.Response): Server response.

        Returns:
            GetLiftPosition.Response: lift_position
        """
        try:
            self.get_logger().info("Lift position requested.")
            response.lift_position = self.plc.lift_position_meter
            return response
        except BaseException as e:
            self.get_logger().error(f"Exception in providing lift position. Error {e}")

    def tf_broadcaster_callback(self, msg: Float32):
        """TF lift broadcaster.

        Args:
            msg (Float32):Lift position.
        """

        position = msg.data
        segment_length = round(position / 3, 3)

        for segment in SEGMENTS:
            t = TransformStamped()

            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = segment[0]
            t.child_frame_id = segment[1]

            # translation in z direction
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = segment_length

            # no rotation
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            self.get_logger().info(
                (
                    f"Publishing lift {segment[0]}-{segment[1]} tf"
                    f" tree:{t.transform}."
                ),
                throttle_duration_sec=THROTTLE_LIFT_POS_LOGGING,
            )
            self.tf_broadcaster.sendTransform(t)

    def _publish_log_message(
        self, log_message: str, response_code: int, error_detail: str
    ):
        """Publisher logger message.

        Args:
            log_message (str): Content of the message.
            response_code (int): Response code as defined in
                                 bautiro_common.bautiro_ros_interfaces.msg.
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

    @property
    def emulation(self):
        """Returns emulation flag (Ros node parameter use_sim_time.").

        Returns:
            bool: Emulation flag.
        """
        return self.get_parameter(name="use_sim_time").value


def main():
    rclpy.init()

    try:
        lift_driver = LiftDriver()
        executor = MultiThreadedExecutor(4)
        executor.add_node(node=lift_driver)
        try:
            executor.spin()

        except KeyboardInterrupt:
            lift_driver.get_logger().fatal("Lift driver keyboard interrupt.")

        except BaseException as e:
            eString = "".join(
                traceback.format_exception(etype=type(e), value=e, tb=e.__traceback__)
            )
            Node(node_name=NODE_NAME).get_logger().fatal(
                f"Lift driver. Unknown exception:{eString}."
            )

        lift_driver.get_logger().fatal("Lift driver shuting down.")
        rclpy.shutdown()

    except Exception as e:
        eString = "".join(
            traceback.format_exception(etype=type(e), value=e, tb=e.__traceback__)
        )
        Node(node_name=NODE_NAME).get_logger().fatal(
            f"Lift driver unknown exception: {eString}."
        )
