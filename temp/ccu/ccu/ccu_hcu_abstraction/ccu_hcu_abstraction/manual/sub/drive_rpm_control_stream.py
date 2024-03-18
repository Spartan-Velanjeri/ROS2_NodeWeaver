# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from typing import Generator, Any
from google.protobuf.empty_pb2 import Empty
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup as CbG
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from ccu_grpc.manual_mode_service_pb2 import DriveRpmControlMessage
from bautiro_developer_functions.core.node_delegator import NodeDelegator


class DriveRpmControlStream(NodeDelegator):

    def __init__(self, node: Node) -> None:
        super().__init__(node)
        self._pub = node.create_publisher(Twist, '/rpm_velocity_controller/cmd_vel_unstamped',
                                          10, callback_group=CbG())

    def drive_rpm_control_stream(self,
                                 request_iterator: Generator[DriveRpmControlMessage, None, Any],
                                 context) -> None:
        self.info("MANUAL: DriveRpmControlStream called by client...")
        for q in request_iterator:
            self.do_pub(q.forward_backward_axis, q.left_right_axis, q.speed_mult)
            if not context.is_active():
                return
        return Empty()

    def do_pub(self, translation: float, rotation: float, speed: float) -> None:
        self.info(f'rpm-msg  Y:{translation:.2f} X:{rotation:.2f} S:{speed:.2f}')
        self._pub.publish(Twist(linear=Vector3(x=translation * max(speed, 0.8)),
                                angular=Vector3(z=rotation)))
