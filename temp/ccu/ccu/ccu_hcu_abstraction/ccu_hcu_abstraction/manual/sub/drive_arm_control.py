# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup as CgB
from bautiro_ros_interfaces.action import MoveHandlingUnitRelativToWorkplane
from ccu_grpc.manual_mode_service_pb2 import DriveArmControlMessage
from bautiro_developer_functions.core.caller import Caller


class DriveArmControl(Caller):

    TYP = MoveHandlingUnitRelativToWorkplane
    NAME = 'fpm_set_move_relative'

    def __init__(self, node: Node, t=TYP, n=NAME) -> None:
        super().__init__(node)
        self.ac = ActionClient(node, t, n, callback_group=CgB())

    def blocking_hu_move(self, q: DriveArmControlMessage, timeout=15, t=TYP, n=NAME) -> None:

        # IMPORTANT! has to be in sync with:  hcu_ccu_interface_definition
        #                                      └─ *.proto
        #                                          └─ enum Direction

        mapping = {0: [-1.0, 0.0],  # LEFT                enum Direction { LEFT = 0;
                   1: [1.0, 0.0],   # RIGHT                                RIGHT = 1;
                   2: [0.0, 1.0],   # FORWARD                              FORWARD = 2;
                   3: [0.0, -1.0]}  # BACKWARD                             BACKWARD = 3; }

        x = q.speed_mult * mapping[q.direction][0]
        y = q.speed_mult * mapping[q.direction][1]

        goal = MoveHandlingUnitRelativToWorkplane.Goal(relative_target_position=[x, y])
        self.call_result(t, n, goal, timeout=timeout, ac=self.ac)
