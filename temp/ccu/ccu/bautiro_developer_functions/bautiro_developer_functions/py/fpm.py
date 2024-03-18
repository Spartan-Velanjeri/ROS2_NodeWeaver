# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from rclpy.node import Node
from bautiro_developer_functions.core.call_action_by_node import call_action_fire_and_forget
from bautiro_ros_interfaces.action import StartFpmSkill


def set_state_rpm_driving(node: Node) -> str:
    return __call_fpm_skill(node, 'bring_hu_down_for_manual_rpm_driving.xml')


def set_state_rpm_fine_positioning(node: Node) -> str:
    return __call_fpm_skill(node, 'bring_hu_under_ceiling_for_manual_drilling.xml')


def __call_fpm_skill(node: Node, skill_xml) -> str:
    return call_action_fire_and_forget(ac_type=StartFpmSkill,
                                       ac_srv_name='/fpm_skill',
                                       goal=StartFpmSkill.Goal(skill_name=skill_xml),
                                       node=node)
