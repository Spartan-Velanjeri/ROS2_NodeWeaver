# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from ccu_grpc.base_layer_service_pb2 import FileStreamingMessage
from typing import Generator, Any
from rclpy.node import Node
from google.protobuf.empty_pb2 import Empty

from ccu_grpc.base_layer_service_pb2_grpc import BaseLayerServiceServicer
from ccu_grpc.base_layer_service_pb2 import (MissionIdRequest, StartMissionResponse,
                                             BaseLayerState, BautiroDescriptionMessage,
                                             BautiroVisualFileRequest,
                                             MissionList, MissionDataResponse,
                                             RobotTransform, DirectoryStreamingResponse,
                                             DeveloperFunctionList, DeveloperFunctionCallResult)
from ccu_util.util import stream_file

from ccu_hcu_abstraction.base.sub.robot_description import RobotDescription
from ccu_hcu_abstraction.base.sub.start_mission import StartMission
from ccu_hcu_abstraction.base.sub.base_layer_state import BaseLayerStateStreamer
from ccu_hcu_abstraction.base.sub.get_mission_list import GetMissionList
from ccu_hcu_abstraction.base.sub.get_mission_data import GetMissionData
from ccu_hcu_abstraction.base.sub.import_mission import ImportMission
from ccu_hcu_abstraction.base.sub.get_obj_file import GetObjectFile
from bautiro_developer_functions.core.developer_functions import DeveloperFunctions


class BaseSrv(BaseLayerServiceServicer):
    """Bridge BaseLayerService to ROS nodes."""

    def __init__(self, node: Node) -> None:
        super().__init__()
        self.bls = BaseLayerStateStreamer(node)
        self.gml = GetMissionList(node)
        self.im = ImportMission(node)
        self.sm = StartMission(node)
        self.gof = GetObjectFile(node)
        self.gmd = GetMissionData(node)
        self.rd = RobotDescription(node)

        self.df = DeveloperFunctions(node)
        self.df.log("Launch Service: 'BaseLayer'")

    def BaseLayerStateStream(self, r: Empty,                                            # noqa N802
                             context) -> Generator[BaseLayerState, None, Any]:
        return self.bls.base_layer_state_stream(context)

    def GetMissionList(self, r: Empty, context) -> MissionList:                         # noqa N802
        result = self.gml.get_mission_list()
        if context.is_active():
            return result

    def ImportMission(self, request_iterator, context) -> DirectoryStreamingResponse:   # noqa E402
        return self.im.import_mission(request_iterator, context)

    def StartMission(self, r: MissionIdRequest, context) -> StartMissionResponse:       # noqa N802
        success = self.sm.start_mission(r.mission_id)
        if context.is_active():
            return StartMissionResponse(success=success)

        return self.sm.start_mission(r.mission_id)

    def GetRoomPlanObjFile(self, mi_id,                                                 # noqa N802
                           context) -> Generator[FileStreamingMessage, None, Any]:
        return self.gof.get_room_plan_obj_file(mi_id, context)

    def GetMissionData(self, r: MissionIdRequest, context) -> MissionDataResponse:      # noqa N802
        result = self.gmd.get_mission_data(r)
        if context.is_active():
            return result

    def StreamMissionData(self, r: Empty,                                               # noqa N802
                          context) -> Generator[MissionDataResponse, None, Any]:
        return self.gmd.stream_mission_data(context)

    def GetBautiroDescription(self, r: Empty, context) -> BautiroDescriptionMessage:    # noqa N802
        return self.rd.get_bautiro_description()

    def GetBautiroMesh(self, r: BautiroVisualFileRequest, ctx) -> FileStreamingMessage: # noqa N802
        self.sm.info('GetBautiroMesh called by client ... ')
        return stream_file(r.file_path, r.file_path, ctx)

    def StreamBautiroTransforms(self, r: Empty,                                         # noqa N802
                                context) -> Generator[RobotTransform, None, Any]:
        yield RobotTransform()

    def GetDeveloperFunctions(self, r: Empty, context) -> DeveloperFunctionList:        # noqa N802
        result = self.df.get_developer_functions()
        if context.is_active():
            return result

    def CallDeveloperFunction(self, r, context) -> DeveloperFunctionCallResult:         # noqa N802
        result = self.df.call_developer_function(r)
        if context.is_active():
            return result
