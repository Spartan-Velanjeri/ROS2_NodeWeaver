# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from time import sleep
from typing import Generator, Any
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup as CbG
from ccu_interfaces.srv import ProtoInRos
from ccu_grpc.base_layer_service_pb2 import MissionIdRequest, MissionDataResponse
from ccu_grpc.util import to_str, inst
from bautiro_developer_functions.core.caller import Caller


class GetMissionData(Caller):

    TYP = ProtoInRos
    NAME = '/get_mission_data'

    def __init__(self, node: Node, t=TYP, n=NAME):
        super().__init__(node)
        self.cl = node.create_client(t, n, callback_group=CbG())

    def _call(self, request: ProtoInRos.Request, t=TYP, n=NAME) -> ProtoInRos.Response:
        return self.call_service_response(t, n, request, client=self.cl)

    def get_mission_data(self, q: MissionIdRequest) -> MissionDataResponse:
        if r := self._call(ProtoInRos.Request(proto=to_str(q))):
            return inst(MissionDataResponse, r.proto)
        return MissionDataResponse()

    def stream_mission_data(self, context) -> Generator[MissionDataResponse, None, Any]:
        while context.is_active():
            if md := self.get_mission_data(MissionIdRequest(mission_id=-7)):  # magic -7
                yield md
            sleep(7)
