# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from rclpy.node import Node

from geometry_msgs.msg import Pose
from bautiro_ros_interfaces.msg import DrillMask, DrillHole
from bautiro_ros_interfaces.action import MoveHandlingUnitRelativToWorkplane as HuMove
from bautiro_ros_interfaces.action import StartFpmSkill

from bautiro_developer_functions.core.call_action_by_node import call_action_fire_and_forget
from bautiro_developer_functions.core.call_action_by_node import call_action_str


def hu_move_relative(node: Node, x: float, y: float) -> str:
    """Up to 3 min blocking send_goal-request until result arrives."""  # TODO progress_cb
    return call_action_str(node=Node,
                           ac_type=HuMove,
                           ac_srv_name='fpm_set_move_relative',
                           goal=HuMove.Goal(relative_target_position=[x, y]),
                           timeout=60.0 * 1)


def start_drill(node: Node, depth: float = 30.0) -> str:

    drill_hole = DrillHole(id='1',                     # string
                           depth=depth,                # double
                           # diameter                  # double
                           pks_pose=Pose(),            # Pose
                           state=DrillHole.UNDRILLED)  # int64

    drill_mask = DrillMask(id='developer_call',
                           pks_pose=Pose(),
                           drill_holes=[drill_hole])

    goal = StartFpmSkill.Goal(skill_name='drill_pattern.xml',
                              drill_masks=[drill_mask])

    return call_action_fire_and_forget(StartFpmSkill, '/fpm_skill', goal, node=node)
