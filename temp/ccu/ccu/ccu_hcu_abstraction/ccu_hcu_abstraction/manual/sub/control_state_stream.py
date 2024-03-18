# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from time import sleep
from typing import Generator, Any
from rclpy.node import Node
from ccu_grpc.manual_mode_service_pb2 import ControlState
from bautiro_developer_functions.core.node_delegator import NodeDelegator


class ControlStateStream(NodeDelegator):

    def __init__(self, node: Node) -> None:
        super().__init__(node)

    def control_state_stream(self, context) -> Generator[ControlState, None, Any]:
        self.info('MANUAL: ControlStateStream called by client...')
        while context.is_active():
            yield ControlState(can_drive_rpm=True,
                               can_raise_lift=True,
                               can_lower_lift=True,
                               can_move_arm=True)
            sleep(1)
        return
