# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup as CbG
from rclpy.action import ActionClient
from bautiro_ros_interfaces.action import StartCcuBt
from ccu_interfaces.srv import GenericPrim
from bautiro_developer_functions.core.caller import Caller


class StartMission(Caller):

    SRV_TYP, SRV_NAME = GenericPrim, '/set_active_mission'
    AC_TYP,  AC_NAME = StartCcuBt,   'ccu_start'

    def start_mission(self, mission_id: int) -> bool:
        if r := self._set_active_mission(mission_id):
            if r and r.data_bool:  # <- success
                self._start_ccu_bt(bt_xml='execute_mission.xml')
                return True
        return False

    def __init__(self, node: Node, st=SRV_TYP, sn=SRV_NAME, at=AC_TYP, an=AC_NAME):
        super().__init__(node)
        self.cl = node.create_client(st, sn, callback_group=CbG())
        self.ac = ActionClient(node, at, an, callback_group=CbG())

    def _set_active_mission(self, mission_id: int, t=SRV_TYP, n=SRV_NAME) -> GenericPrim.Response:
        return self.call_service_response(t, n, GenericPrim.Request(data_int=mission_id),
                                          client=self.cl)

    def _start_ccu_bt(self, bt_xml: str, t=AC_TYP, n=AC_NAME) -> bool:
        return self.call_fire_and_forget(t, n, StartCcuBt.Goal(start=True, bt_xml=bt_xml),
                                         ac=self.ac)
