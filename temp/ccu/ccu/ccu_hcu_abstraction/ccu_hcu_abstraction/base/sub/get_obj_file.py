# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from typing import Generator, Any
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup as CbG
from ccu_grpc.base_layer_service_pb2 import FileStreamingMessage, MissionIdRequest
from bautiro_developer_functions.core.node_delegator import NodeDelegator
from ccu_interfaces.srv import GenericPrim
from ccu_util.util import stream_file


class GetObjectFile(NodeDelegator):

    def __init__(self, node: Node):
        super().__init__(node)
        self.client = node.create_client(GenericPrim, '/get_obj_file', callback_group=CbG())

    def get_room_plan_obj_file(self, r: MissionIdRequest,
                               context) -> Generator[FileStreamingMessage, None, Any]:
        self.info('GetRoomPlanObjFile called by client ... ')
        self.info(f'call begin : {self.client.srv_name}')

        if not self.client.wait_for_service(timeout_sec=3.0):
            self.warn(f'service not available: {self.client.srv_name}')
            return

        r: GenericPrim.Response = self.client.call(GenericPrim.Request(data_int=r.mission_id))
        self.info(f'call   end : {self.client.srv_name}')
        name_path_str = r.data_str

        if name_path_str and (':' in name_path_str):  # obj_file  = 'name.obj:/path/to/1.obj'
            name, path = name_path_str.split(':', 1)
        else:
            name = path = name_path_str
        return stream_file(name, path, context)
