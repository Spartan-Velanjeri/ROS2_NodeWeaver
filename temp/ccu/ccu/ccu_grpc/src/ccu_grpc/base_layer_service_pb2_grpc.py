# Generated by the gRPC Python protocol compiler plugin. DO NOT EDIT!
"""Client and server classes corresponding to protobuf-defined services."""
import grpc

import ccu_grpc.base_layer_service_pb2 as base__layer__service__pb2
from google.protobuf import empty_pb2 as google_dot_protobuf_dot_empty__pb2


class BaseLayerServiceStub(object):
    """Missing associated documentation comment in .proto file."""

    def __init__(self, channel):
        """Constructor.

        Args:
            channel: A grpc.Channel.
        """
        self.BaseLayerStateStream = channel.unary_stream(
                '/google.protobuf.BaseLayerService/BaseLayerStateStream',
                request_serializer=google_dot_protobuf_dot_empty__pb2.Empty.SerializeToString,
                response_deserializer=base__layer__service__pb2.BaseLayerState.FromString,
                )
        self.GetMissionList = channel.unary_unary(
                '/google.protobuf.BaseLayerService/GetMissionList',
                request_serializer=google_dot_protobuf_dot_empty__pb2.Empty.SerializeToString,
                response_deserializer=base__layer__service__pb2.MissionList.FromString,
                )
        self.GetRoomPlanObjFile = channel.unary_stream(
                '/google.protobuf.BaseLayerService/GetRoomPlanObjFile',
                request_serializer=base__layer__service__pb2.MissionIdRequest.SerializeToString,
                response_deserializer=base__layer__service__pb2.FileStreamingMessage.FromString,
                )
        self.GetMissionData = channel.unary_unary(
                '/google.protobuf.BaseLayerService/GetMissionData',
                request_serializer=base__layer__service__pb2.MissionIdRequest.SerializeToString,
                response_deserializer=base__layer__service__pb2.MissionDataResponse.FromString,
                )
        self.StreamMissionData = channel.unary_stream(
                '/google.protobuf.BaseLayerService/StreamMissionData',
                request_serializer=google_dot_protobuf_dot_empty__pb2.Empty.SerializeToString,
                response_deserializer=base__layer__service__pb2.MissionDataResponse.FromString,
                )
        self.StartMission = channel.unary_unary(
                '/google.protobuf.BaseLayerService/StartMission',
                request_serializer=base__layer__service__pb2.MissionIdRequest.SerializeToString,
                response_deserializer=base__layer__service__pb2.StartMissionResponse.FromString,
                )
        self.ImportMission = channel.stream_unary(
                '/google.protobuf.BaseLayerService/ImportMission',
                request_serializer=base__layer__service__pb2.DirectoryStreamingMessage.SerializeToString,
                response_deserializer=base__layer__service__pb2.DirectoryStreamingResponse.FromString,
                )
        self.GetBautiroDescription = channel.unary_unary(
                '/google.protobuf.BaseLayerService/GetBautiroDescription',
                request_serializer=google_dot_protobuf_dot_empty__pb2.Empty.SerializeToString,
                response_deserializer=base__layer__service__pb2.BautiroDescriptionMessage.FromString,
                )
        self.GetBautiroMesh = channel.unary_stream(
                '/google.protobuf.BaseLayerService/GetBautiroMesh',
                request_serializer=base__layer__service__pb2.BautiroVisualFileRequest.SerializeToString,
                response_deserializer=base__layer__service__pb2.FileStreamingMessage.FromString,
                )
        self.StreamBautiroTransforms = channel.unary_stream(
                '/google.protobuf.BaseLayerService/StreamBautiroTransforms',
                request_serializer=google_dot_protobuf_dot_empty__pb2.Empty.SerializeToString,
                response_deserializer=base__layer__service__pb2.RobotTransform.FromString,
                )
        self.GetDeveloperFunctions = channel.unary_unary(
                '/google.protobuf.BaseLayerService/GetDeveloperFunctions',
                request_serializer=google_dot_protobuf_dot_empty__pb2.Empty.SerializeToString,
                response_deserializer=base__layer__service__pb2.DeveloperFunctionList.FromString,
                )
        self.CallDeveloperFunction = channel.unary_unary(
                '/google.protobuf.BaseLayerService/CallDeveloperFunction',
                request_serializer=base__layer__service__pb2.DeveloperFunctionDefinition.SerializeToString,
                response_deserializer=base__layer__service__pb2.DeveloperFunctionCallResult.FromString,
                )


class BaseLayerServiceServicer(object):
    """Missing associated documentation comment in .proto file."""

    def BaseLayerStateStream(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def GetMissionList(self, request, context):
        """get the list of available missions
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def GetRoomPlanObjFile(self, request, context):
        """get a roomplan for a specific mission
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def GetMissionData(self, request, context):
        """get data for a specific mission
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def StreamMissionData(self, request, context):
        """stream changes to mission data - a.k.a. "Tracking mode"
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def StartMission(self, request, context):
        """start a mission and stream mission events
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def ImportMission(self, request_iterator, context):
        """HCU sends files contained in a directory to CCU in order to create a new mission
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def GetBautiroDescription(self, request, context):
        """get a description of the bautiro transform tree
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def GetBautiroMesh(self, request, context):
        """download a visual mesh file
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def StreamBautiroTransforms(self, request, context):
        """stream live transforms
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def GetDeveloperFunctions(self, request, context):
        """get all developer functions
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def CallDeveloperFunction(self, request, context):
        """call a developer function
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')


def add_BaseLayerServiceServicer_to_server(servicer, server):
    rpc_method_handlers = {
            'BaseLayerStateStream': grpc.unary_stream_rpc_method_handler(
                    servicer.BaseLayerStateStream,
                    request_deserializer=google_dot_protobuf_dot_empty__pb2.Empty.FromString,
                    response_serializer=base__layer__service__pb2.BaseLayerState.SerializeToString,
            ),
            'GetMissionList': grpc.unary_unary_rpc_method_handler(
                    servicer.GetMissionList,
                    request_deserializer=google_dot_protobuf_dot_empty__pb2.Empty.FromString,
                    response_serializer=base__layer__service__pb2.MissionList.SerializeToString,
            ),
            'GetRoomPlanObjFile': grpc.unary_stream_rpc_method_handler(
                    servicer.GetRoomPlanObjFile,
                    request_deserializer=base__layer__service__pb2.MissionIdRequest.FromString,
                    response_serializer=base__layer__service__pb2.FileStreamingMessage.SerializeToString,
            ),
            'GetMissionData': grpc.unary_unary_rpc_method_handler(
                    servicer.GetMissionData,
                    request_deserializer=base__layer__service__pb2.MissionIdRequest.FromString,
                    response_serializer=base__layer__service__pb2.MissionDataResponse.SerializeToString,
            ),
            'StreamMissionData': grpc.unary_stream_rpc_method_handler(
                    servicer.StreamMissionData,
                    request_deserializer=google_dot_protobuf_dot_empty__pb2.Empty.FromString,
                    response_serializer=base__layer__service__pb2.MissionDataResponse.SerializeToString,
            ),
            'StartMission': grpc.unary_unary_rpc_method_handler(
                    servicer.StartMission,
                    request_deserializer=base__layer__service__pb2.MissionIdRequest.FromString,
                    response_serializer=base__layer__service__pb2.StartMissionResponse.SerializeToString,
            ),
            'ImportMission': grpc.stream_unary_rpc_method_handler(
                    servicer.ImportMission,
                    request_deserializer=base__layer__service__pb2.DirectoryStreamingMessage.FromString,
                    response_serializer=base__layer__service__pb2.DirectoryStreamingResponse.SerializeToString,
            ),
            'GetBautiroDescription': grpc.unary_unary_rpc_method_handler(
                    servicer.GetBautiroDescription,
                    request_deserializer=google_dot_protobuf_dot_empty__pb2.Empty.FromString,
                    response_serializer=base__layer__service__pb2.BautiroDescriptionMessage.SerializeToString,
            ),
            'GetBautiroMesh': grpc.unary_stream_rpc_method_handler(
                    servicer.GetBautiroMesh,
                    request_deserializer=base__layer__service__pb2.BautiroVisualFileRequest.FromString,
                    response_serializer=base__layer__service__pb2.FileStreamingMessage.SerializeToString,
            ),
            'StreamBautiroTransforms': grpc.unary_stream_rpc_method_handler(
                    servicer.StreamBautiroTransforms,
                    request_deserializer=google_dot_protobuf_dot_empty__pb2.Empty.FromString,
                    response_serializer=base__layer__service__pb2.RobotTransform.SerializeToString,
            ),
            'GetDeveloperFunctions': grpc.unary_unary_rpc_method_handler(
                    servicer.GetDeveloperFunctions,
                    request_deserializer=google_dot_protobuf_dot_empty__pb2.Empty.FromString,
                    response_serializer=base__layer__service__pb2.DeveloperFunctionList.SerializeToString,
            ),
            'CallDeveloperFunction': grpc.unary_unary_rpc_method_handler(
                    servicer.CallDeveloperFunction,
                    request_deserializer=base__layer__service__pb2.DeveloperFunctionDefinition.FromString,
                    response_serializer=base__layer__service__pb2.DeveloperFunctionCallResult.SerializeToString,
            ),
    }
    generic_handler = grpc.method_handlers_generic_handler(
            'google.protobuf.BaseLayerService', rpc_method_handlers)
    server.add_generic_rpc_handlers((generic_handler,))


 # This class is part of an EXPERIMENTAL API.
class BaseLayerService(object):
    """Missing associated documentation comment in .proto file."""

    @staticmethod
    def BaseLayerStateStream(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_stream(request, target, '/google.protobuf.BaseLayerService/BaseLayerStateStream',
            google_dot_protobuf_dot_empty__pb2.Empty.SerializeToString,
            base__layer__service__pb2.BaseLayerState.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def GetMissionList(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/google.protobuf.BaseLayerService/GetMissionList',
            google_dot_protobuf_dot_empty__pb2.Empty.SerializeToString,
            base__layer__service__pb2.MissionList.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def GetRoomPlanObjFile(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_stream(request, target, '/google.protobuf.BaseLayerService/GetRoomPlanObjFile',
            base__layer__service__pb2.MissionIdRequest.SerializeToString,
            base__layer__service__pb2.FileStreamingMessage.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def GetMissionData(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/google.protobuf.BaseLayerService/GetMissionData',
            base__layer__service__pb2.MissionIdRequest.SerializeToString,
            base__layer__service__pb2.MissionDataResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def StreamMissionData(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_stream(request, target, '/google.protobuf.BaseLayerService/StreamMissionData',
            google_dot_protobuf_dot_empty__pb2.Empty.SerializeToString,
            base__layer__service__pb2.MissionDataResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def StartMission(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/google.protobuf.BaseLayerService/StartMission',
            base__layer__service__pb2.MissionIdRequest.SerializeToString,
            base__layer__service__pb2.StartMissionResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def ImportMission(request_iterator,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.stream_unary(request_iterator, target, '/google.protobuf.BaseLayerService/ImportMission',
            base__layer__service__pb2.DirectoryStreamingMessage.SerializeToString,
            base__layer__service__pb2.DirectoryStreamingResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def GetBautiroDescription(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/google.protobuf.BaseLayerService/GetBautiroDescription',
            google_dot_protobuf_dot_empty__pb2.Empty.SerializeToString,
            base__layer__service__pb2.BautiroDescriptionMessage.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def GetBautiroMesh(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_stream(request, target, '/google.protobuf.BaseLayerService/GetBautiroMesh',
            base__layer__service__pb2.BautiroVisualFileRequest.SerializeToString,
            base__layer__service__pb2.FileStreamingMessage.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def StreamBautiroTransforms(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_stream(request, target, '/google.protobuf.BaseLayerService/StreamBautiroTransforms',
            google_dot_protobuf_dot_empty__pb2.Empty.SerializeToString,
            base__layer__service__pb2.RobotTransform.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def GetDeveloperFunctions(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/google.protobuf.BaseLayerService/GetDeveloperFunctions',
            google_dot_protobuf_dot_empty__pb2.Empty.SerializeToString,
            base__layer__service__pb2.DeveloperFunctionList.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def CallDeveloperFunction(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/google.protobuf.BaseLayerService/CallDeveloperFunction',
            base__layer__service__pb2.DeveloperFunctionDefinition.SerializeToString,
            base__layer__service__pb2.DeveloperFunctionCallResult.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)