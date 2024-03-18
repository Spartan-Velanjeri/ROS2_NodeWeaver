# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from rclpy.node import Node
from bautiro_ros_interfaces.action import MoveLiftAbsolute
from bautiro_developer_functions.core.call_action_by_node import call_action_str
from bautiro_developer_functions.core.lib import MIN, MAX, STEP, limit


def move_lift_to_3_00(node: Node) -> str:
    return move_lift_to_height(node, target_height=3.0)


def move_lift_to_2_50(node: Node) -> str:
    return move_lift_to_height(node, target_height=2.5)


def move_lift_to_2_00(node: Node) -> str:
    return move_lift_to_height(node, target_height=2.0)


def move_lift_to_1_50(node: Node) -> str:
    return move_lift_to_height(node, target_height=1.5)


MOVE_LIFT_TO_HEIGHT = {'target_height': {MIN:  0.0, MAX: 5.0, STEP: 0.01}}


def move_lift_to_height(node: Node,
                        target_height: float = 1.47,   # si unit --> 1,47 meter
                        timeout: float = 60.0,
                        ) -> str:
    th = limit(target_height, 'target_height', MOVE_LIFT_TO_HEIGHT)

    return call_action_str(MoveLiftAbsolute,
                           '/move_lift_absolute',
                           MoveLiftAbsolute.Goal(requested_target_lift_level=th),
                           timeout=timeout,
                           node=node)


def move_lift_to_height_via_driver(node: Node,
                                   target_height: float = 1.47,   # si unit --> 1,47 meter
                                   speed: float = 1.0,
                                   ) -> str:
    return "Not Implemented(move_lift_to_height_via_driver)"
