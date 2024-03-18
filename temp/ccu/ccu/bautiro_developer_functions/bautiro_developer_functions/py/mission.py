# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from rclpy.node import Node
from std_srvs.srv import Trigger
from bautiro_ros_interfaces.action import StartCcuBt
from ccu_interfaces.srv import GenericPrim
from bautiro_developer_functions.core.call_srv import call_service_str, call_service_response
from bautiro_developer_functions.core.call_action_by_node import call_action_fire_and_forget


def set_active_mission(node: Node, mission_index_to_set: int = 0) -> str:
    return call_service_str(node,
                            GenericPrim,
                            '/set_active_mission',
                            GenericPrim.Request(data_int=mission_index_to_set))


def unset_active_mission(node: Node) -> str:
    return call_service_str(node,
                            Trigger,
                            '/unset_active_mission',
                            Trigger.Request())


###################################################################################################
#                                                                                                 #
#  start ccu_bt                                                                                   #
#                                                                                                 #
###################################################################################################

def start_mission(node: Node):
    pass


def _start_mission_n(node: Node, mission_id, bt_xml: str):
    if r := call_service_response(node, GenericPrim, '/set_active_mission',
                                  GenericPrim.Request(data_int=mission_id)):
        if r and r.data_bool:  # <- success
            return call_action_fire_and_forget(StartCcuBt,
                                               'ccu_start',
                                               StartCcuBt.Goal(
                                                   start=True, bt_xml=bt_xml),
                                               node=node)
    return "Failed"


START_CCU_BT_XML_ENUM = {
    1: 'execute_mission.xml',
    2: 'ccu_data_service_test.xml',
    3: 'ccu_main_feedback.xml',
    4: 'motion_cluster.xml',
}
START_CCU_BT_INFO = f"start ccu behavior tree. Enum\n {START_CCU_BT_XML_ENUM}"


def start_ccu_bt(node: Node, bt_xml_enum: int = 0) -> bool:

    if bt_xml_enum not in START_CCU_BT_XML_ENUM:
        return f"{bt_xml_enum} not in {START_CCU_BT_XML_ENUM.values()}[{START_CCU_BT_XML_ENUM}]"

    bt_xml = START_CCU_BT_XML_ENUM[bt_xml_enum]

    return call_action_fire_and_forget(StartCcuBt,
                                       'ccu_start',
                                       StartCcuBt.Goal(
                                           start=True, bt_xml=bt_xml),
                                       node=node)
