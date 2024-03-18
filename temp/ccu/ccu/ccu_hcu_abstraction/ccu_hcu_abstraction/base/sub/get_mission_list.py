# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from google.protobuf.empty_pb2 import Empty
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup as CbG
from ccu_grpc.base_layer_service_pb2 import MissionList
from ccu_grpc.util import inst
from ccu_interfaces.srv import ProtoInRos
from bautiro_developer_functions.core.caller import Caller


class GetMissionList(Caller):

    TYP = ProtoInRos
    NAME = '/get_mission_list'

    def __init__(self, node: Node, typ=TYP, name=NAME):
        super().__init__(node)
        self.cl = node.create_client(typ, name, callback_group=CbG())

    def _call(self, typ=TYP, name=NAME) -> ProtoInRos.Response:
        return self.call_service_response(typ, name, typ.Request(), client=self.cl)

    def get_mission_list(self) -> MissionList:
        if r := self._call():
            return inst(MissionList, r.proto)
        return MissionList()
