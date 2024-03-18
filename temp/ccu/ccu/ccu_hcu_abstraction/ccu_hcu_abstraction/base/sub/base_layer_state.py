# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.


from typing import Generator, Any
from rclpy.node import Node

from ccu_grpc.base_layer_service_pb2 import BaseLayerState, MissionMetaData
from ccu_grpc.util import inst
from bautiro_developer_functions.core.node_delegator import NodeDelegator
from ccu_interfaces.msg import ProtoString as Proto


from threading import Event


class BaseLayerStateStreamer(NodeDelegator):

    def __init__(self, node: Node) -> None:
        super().__init__(node)
        self.node.create_subscription(Proto, 'current_active_mission_metadata', self.__cb, 10)

        self._old_index = -2
        self.act_mission: MissionMetaData = MissionMetaData(id=-1)  # indicates 'not active'
        self.act_mi_updated = Event()

    def __cb(self, proto_string: Proto) -> None:
        mmd = inst(MissionMetaData, proto_string.proto)
        if mmd.id != self._old_index:  # log on change Only
            self.info(f"Changed 'current_active_mission_metadata': {mmd.id}")
        self.update(mmd)  # every second dataservice updates
        self._old_index = mmd.id

    def update(self, new_mission: MissionMetaData) -> None:
        self.act_mission = new_mission
        self.act_mi_updated.set()

    def base_layer_state_stream(self, context) -> Generator[BaseLayerState, None, Any]:
        self.info("Start 'BaseLayerState' message-stream")
        while context.is_active():
            if self.act_mi_updated.wait(timeout=3.0):
                yield BaseLayerState(can_start_mission=self.act_mission.id < 1,  # nothing active
                                     can_switch_to_manual_mode=True,
                                     mission_in_progress=self.act_mission)
                self.act_mi_updated.clear()
            else:
                yield BaseLayerState(can_start_mission=False,
                                     can_switch_to_manual_mode=True)
