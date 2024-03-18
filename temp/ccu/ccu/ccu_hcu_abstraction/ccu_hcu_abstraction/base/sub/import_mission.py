# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup as CbG
from ccu_interfaces.srv import GenericPrim
from bautiro_developer_functions.core.caller import Caller
from ccu_grpc.base_layer_service_pb2 import DirectoryStreamingResponse
from ccu_util.util import receive_directory_stream


class ImportMission(Caller):

    __DEFAULT_TRANSFER_DIR = '/tmp/bautiro_transfer_directory'

    typ = GenericPrim
    name = '/import_mission'

    def __init__(self, node: Node):
        super().__init__(node)
        self.cl = node.create_client(self.typ, self.name, callback_group=CbG())

    def _import_mission(self, q: GenericPrim.Request) -> GenericPrim.Response:
        return self.call_service_response(self.typ, self.name, q, client=self.cl)

    def import_mission(self, request_iterator, context,
                       tmp_dir: str = __DEFAULT_TRANSFER_DIR,) -> DirectoryStreamingResponse:
        success = False
        if receive_directory_stream(tmp_dir, request_iterator, context) > 0:
            if number_of_files := self._import_mission(GenericPrim.Request(data_str=tmp_dir)):
                success = number_of_files.data_int > 0
        if context.is_active():
            return DirectoryStreamingResponse(success=success)
