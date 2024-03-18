# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: base_layer_service.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from google.protobuf import empty_pb2 as google_dot_protobuf_dot_empty__pb2
from google.protobuf import timestamp_pb2 as google_dot_protobuf_dot_timestamp__pb2
import ccu_grpc.common_types_pb2 as common__types__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x18\x62\x61se_layer_service.proto\x12\x0fgoogle.protobuf\x1a\x1bgoogle/protobuf/empty.proto\x1a\x1fgoogle/protobuf/timestamp.proto\x1a\x12\x63ommon_types.proto\"\xaa\x01\n\x0e\x42\x61seLayerState\x12!\n\x19\x63\x61n_switch_to_manual_mode\x18\x01 \x01(\x08\x12\x19\n\x11\x63\x61n_start_mission\x18\x02 \x01(\x08\x12\x42\n\x13mission_in_progress\x18\n \x01(\x0b\x32 .google.protobuf.MissionMetaDataH\x00\x88\x01\x01\x42\x16\n\x14_mission_in_progress\"&\n\x10MissionIdRequest\x12\x12\n\nmission_id\x18\x01 \x01(\x03\"\'\n\x14StartMissionResponse\x12\x0f\n\x07success\x18\x01 \x01(\x08\"d\n\x0bMissionList\x12\x32\n\x08missions\x18\x01 \x03(\x0b\x32 .google.protobuf.MissionMetaData\x12!\n\x19\x63urrent_active_mission_id\x18\x02 \x01(\x03\"\x8b\x01\n\x0fMissionMetaData\x12\n\n\x02id\x18\x01 \x01(\x03\x12\x0c\n\x04name\x18\x02 \x01(\t\x12\x16\n\x0epredecessor_id\x18\x03 \x01(\x03\x12/\n\x0bimport_date\x18\x04 \x01(\x0b\x32\x1a.google.protobuf.Timestamp\x12\x15\n\rroom_plan_md5\x18\x05 \x01(\t\"z\n\x13MissionDataResponse\x12(\n\tnode_tree\x18\x01 \x03(\x0b\x32\x15.google.protobuf.Node\x12\"\n\x04jobs\x18\x02 \x03(\x0b\x32\x14.google.protobuf.Job\x12\x15\n\rroom_plan_md5\x18\x03 \x01(\t\"\xb2\x02\n\x04Node\x12\'\n\x08\x63hildren\x18\x01 \x03(\x0b\x32\x15.google.protobuf.Node\x12\x33\n\x0flocal_transform\x18\x02 \x01(\x0b\x32\x1a.google.protobuf.Matrix4x4\x12!\n\x19name_unique_per_node_tree\x18\x03 \x01(\t\x12\x0e\n\x06series\x18\x04 \x01(\t\x12\x41\n\x11\x66\x61stener_template\x18\x05 \x01(\x0b\x32!.google.protobuf.FastenerTemplateH\x00\x88\x01\x01\x12$\n\x17parent_node_name_unique\x18\x06 \x01(\tH\x01\x88\x01\x01\x42\x14\n\x12_fastener_templateB\x1a\n\x18_parent_node_name_unique\"\x9a\x01\n\x10\x46\x61stenerTemplate\x12/\n\x0b\x64rill_masks\x18\x01 \x03(\x0b\x32\x1a.google.protobuf.DrillMask\x12!\n\x19name_unique_per_node_tree\x18\x02 \x01(\t\x12\x11\n\ttype_guid\x18\x03 \x01(\t\x12\x1f\n\x17parent_node_name_unique\x18\x04 \x01(\t\"\xc2\x01\n\tDrillMask\x12\x33\n\x0flocal_transform\x18\x01 \x01(\x0b\x32\x1a.google.protobuf.Matrix4x4\x12!\n\x19name_unique_per_node_tree\x18\x02 \x01(\t\x12/\n\x0b\x64rill_holes\x18\x03 \x03(\x0b\x32\x1a.google.protobuf.DrillHole\x12,\n$parent_fastener_template_name_unique\x18\x04 \x01(\t\"\xe8\x02\n\tDrillHole\x12\x33\n\x0flocal_transform\x18\x01 \x01(\x0b\x32\x1a.google.protobuf.Matrix4x4\x12!\n\x19name_unique_per_drillmask\x18\x02 \x01(\t\x12\x38\n\x05state\x18\x03 \x01(\x0e\x32).google.protobuf.DrillHole.DrillHoleState\x12)\n\x1cparent_drillmask_name_unique\x18\x04 \x01(\tH\x00\x88\x01\x01\x12\x0b\n\x03\x64_x\x18\x05 \x01(\x01\x12\x0b\n\x03\x64_y\x18\x06 \x01(\x01\"c\n\x0e\x44rillHoleState\x12\r\n\tUNDEFINED\x10\x00\x12\r\n\tUNDRILLED\x10\x01\x12\r\n\tSCHEDULED\x10\x02\x12\x0c\n\x08\x46INISHED\x10\x03\x12\n\n\x06IGNORE\x10\x04\x12\n\n\x06\x46\x41ILED\x10\x05\x42\x1f\n\x1d_parent_drillmask_name_unique\"\xfb\x01\n\x03Job\x12\x11\n\tid_unique\x18\x01 \x01(\t\x12\x36\n\x12\x61\x62solute_transform\x18\x02 \x01(\x0b\x32\x1a.google.protobuf.Matrix4x4\x12\x0f\n\x07\x65nabled\x18\x03 \x01(\x08\x12.\n\tdrill_job\x18\n \x01(\x0b\x32\x19.google.protobuf.DrillJobH\x00\x12,\n\x08move_job\x18\x0b \x01(\x0b\x32\x18.google.protobuf.MoveJobH\x00\x12\x32\n\x0bmeasure_job\x18\x0c \x01(\x0b\x32\x1b.google.protobuf.MeasureJobH\x00\x42\x06\n\x04\x64\x61ta\"#\n\x08\x44rillJob\x12\x17\n\x0f\x64rillmask_names\x18\x01 \x03(\t\"\t\n\x07MoveJob\"\x0c\n\nMeasureJob\"L\n\x0c\x46ileMetaData\x12\x11\n\tfile_name\x18\x01 \x01(\t\x12\x17\n\x0f\x66ile_size_bytes\x18\x02 \x01(\x03\x12\x10\n\x08md5_hash\x18\x03 \x01(\t\"\xbb\x01\n\x14\x46ileStreamingMessage\x12\x37\n\x0e\x66ile_meta_data\x18\x02 \x01(\x0b\x32\x1d.google.protobuf.FileMetaDataH\x00\x12\x19\n\x0f\x66ile_chunk_data\x18\x03 \x01(\x0cH\x00\x12G\n\x0e\x66ile_completed\x18\x04 \x01(\x0b\x32-.google.protobuf.FileTransferCompletedMessageH\x00\x42\x06\n\x04\x64\x61ta\"1\n\x1c\x46ileTransferCompletedMessage\x12\x11\n\tfile_name\x18\x01 \x01(\t\"\xe9\x01\n\x19\x44irectoryStreamingMessage\x12;\n\rdir_meta_data\x18\x01 \x01(\x0b\x32\".google.protobuf.DirectoryMetaDataH\x00\x12:\n\tfile_data\x18\x02 \x01(\x0b\x32%.google.protobuf.FileStreamingMessageH\x00\x12K\n\rdir_completed\x18\x03 \x01(\x0b\x32\x32.google.protobuf.DirectoryTransferCompletedMessageH\x00\x42\x06\n\x04\x64\x61ta\"I\n\x11\x44irectoryMetaData\x12\x16\n\x0e\x64irectory_name\x18\x01 \x01(\t\x12\x1c\n\x14\x64irectory_file_count\x18\x02 \x01(\x03\"#\n!DirectoryTransferCompletedMessage\"-\n\x1a\x44irectoryStreamingResponse\x12\x0f\n\x07success\x18\x01 \x01(\x08\"R\n\x15\x44\x65veloperFunctionList\x12\x39\n\x07sources\x18\x01 \x03(\x0b\x32(.google.protobuf.DeveloperFunctionSource\"h\n\x17\x44\x65veloperFunctionSource\x12\x0c\n\x04name\x18\x01 \x01(\t\x12?\n\tfunctions\x18\x02 \x03(\x0b\x32,.google.protobuf.DeveloperFunctionDefinition\"z\n\x1b\x44\x65veloperFunctionDefinition\x12\x0c\n\x04name\x18\x01 \x01(\t\x12\x13\n\x0b\x64\x65scription\x18\x02 \x01(\t\x12\x38\n\nparameters\x18\x03 \x03(\x0b\x32$.google.protobuf.ParameterDefinition\"\x93\x01\n\x13ParameterDefinition\x12\x39\n\x0cparam_number\x18\x01 \x01(\x0b\x32!.google.protobuf.NumericParameterH\x00\x12\x34\n\nparam_bool\x18\x02 \x01(\x0b\x32\x1e.google.protobuf.BoolParameterH\x00\x42\x0b\n\tparameter\"t\n\x10NumericParameter\x12\x12\n\nparam_name\x18\x01 \x01(\t\x12\x13\n\x0bparam_value\x18\x02 \x01(\x01\x12\x11\n\tmin_value\x18\x03 \x01(\x01\x12\x11\n\tmax_value\x18\x04 \x01(\x01\x12\x11\n\tstep_size\x18\x05 \x01(\x01\"8\n\rBoolParameter\x12\x12\n\nparam_name\x18\x01 \x01(\t\x12\x13\n\x0bparam_value\x18\x02 \x01(\x08\".\n\x1b\x44\x65veloperFunctionCallResult\x12\x0f\n\x07message\x18\x01 \x01(\t\"L\n\x19\x42\x61utiroDescriptionMessage\x12/\n\x05links\x18\x01 \x03(\x0b\x32 .google.protobuf.BautiroUrdfLink\"\xaa\x01\n\x0f\x42\x61utiroUrdfLink\x12\x0c\n\x04name\x18\x01 \x01(\t\x12\x0e\n\x06parent\x18\x02 \x01(\t\x12@\n\x1ctransform_relative_to_parent\x18\x03 \x01(\x0b\x32\x1a.google.protobuf.Matrix4x4\x12\x37\n\x07visuals\x18\x04 \x03(\x0b\x32&.google.protobuf.BautiroUrdfVisualMesh\"w\n\x15\x42\x61utiroUrdfVisualMesh\x12\x16\n\x0emesh_file_path\x18\x01 \x01(\t\x12\x1a\n\x12mesh_file_md5_hash\x18\x02 \x01(\t\x12*\n\x06offset\x18\x03 \x01(\x0b\x32\x1a.google.protobuf.Matrix4x4\"-\n\x18\x42\x61utiroVisualFileRequest\x12\x11\n\tfile_path\x18\x01 \x01(\t\"H\n\x0eRobotTransform\x12\x36\n\x05links\x18\x01 \x03(\x0b\x32\'.google.protobuf.UpdateTransformMessage\"l\n\x16UpdateTransformMessage\x12\x10\n\x08\x63hild_id\x18\x01 \x01(\t\x12@\n\x1ctransform_relative_to_parent\x18\x02 \x01(\x0b\x32\x1a.google.protobuf.Matrix4x42\xec\x08\n\x10\x42\x61seLayerService\x12Q\n\x14\x42\x61seLayerStateStream\x12\x16.google.protobuf.Empty\x1a\x1f.google.protobuf.BaseLayerState0\x01\x12\x46\n\x0eGetMissionList\x12\x16.google.protobuf.Empty\x1a\x1c.google.protobuf.MissionList\x12`\n\x12GetRoomPlanObjFile\x12!.google.protobuf.MissionIdRequest\x1a%.google.protobuf.FileStreamingMessage0\x01\x12Y\n\x0eGetMissionData\x12!.google.protobuf.MissionIdRequest\x1a$.google.protobuf.MissionDataResponse\x12S\n\x11StreamMissionData\x12\x16.google.protobuf.Empty\x1a$.google.protobuf.MissionDataResponse0\x01\x12X\n\x0cStartMission\x12!.google.protobuf.MissionIdRequest\x1a%.google.protobuf.StartMissionResponse\x12j\n\rImportMission\x12*.google.protobuf.DirectoryStreamingMessage\x1a+.google.protobuf.DirectoryStreamingResponse(\x01\x12[\n\x15GetBautiroDescription\x12\x16.google.protobuf.Empty\x1a*.google.protobuf.BautiroDescriptionMessage\x12\x64\n\x0eGetBautiroMesh\x12).google.protobuf.BautiroVisualFileRequest\x1a%.google.protobuf.FileStreamingMessage0\x01\x12T\n\x17StreamBautiroTransforms\x12\x16.google.protobuf.Empty\x1a\x1f.google.protobuf.RobotTransform0\x01\x12W\n\x15GetDeveloperFunctions\x12\x16.google.protobuf.Empty\x1a&.google.protobuf.DeveloperFunctionList\x12s\n\x15\x43\x61llDeveloperFunction\x12,.google.protobuf.DeveloperFunctionDefinition\x1a,.google.protobuf.DeveloperFunctionCallResultB\x11\xaa\x02\x0e\x42\x61utiroGrpcLibb\x06proto3')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'base_layer_service_pb2', _globals)
if _descriptor._USE_C_DESCRIPTORS == False:
  DESCRIPTOR._options = None
  DESCRIPTOR._serialized_options = b'\252\002\016BautiroGrpcLib'
  _globals['_BASELAYERSTATE']._serialized_start=128
  _globals['_BASELAYERSTATE']._serialized_end=298
  _globals['_MISSIONIDREQUEST']._serialized_start=300
  _globals['_MISSIONIDREQUEST']._serialized_end=338
  _globals['_STARTMISSIONRESPONSE']._serialized_start=340
  _globals['_STARTMISSIONRESPONSE']._serialized_end=379
  _globals['_MISSIONLIST']._serialized_start=381
  _globals['_MISSIONLIST']._serialized_end=481
  _globals['_MISSIONMETADATA']._serialized_start=484
  _globals['_MISSIONMETADATA']._serialized_end=623
  _globals['_MISSIONDATARESPONSE']._serialized_start=625
  _globals['_MISSIONDATARESPONSE']._serialized_end=747
  _globals['_NODE']._serialized_start=750
  _globals['_NODE']._serialized_end=1056
  _globals['_FASTENERTEMPLATE']._serialized_start=1059
  _globals['_FASTENERTEMPLATE']._serialized_end=1213
  _globals['_DRILLMASK']._serialized_start=1216
  _globals['_DRILLMASK']._serialized_end=1410
  _globals['_DRILLHOLE']._serialized_start=1413
  _globals['_DRILLHOLE']._serialized_end=1773
  _globals['_DRILLHOLE_DRILLHOLESTATE']._serialized_start=1641
  _globals['_DRILLHOLE_DRILLHOLESTATE']._serialized_end=1740
  _globals['_JOB']._serialized_start=1776
  _globals['_JOB']._serialized_end=2027
  _globals['_DRILLJOB']._serialized_start=2029
  _globals['_DRILLJOB']._serialized_end=2064
  _globals['_MOVEJOB']._serialized_start=2066
  _globals['_MOVEJOB']._serialized_end=2075
  _globals['_MEASUREJOB']._serialized_start=2077
  _globals['_MEASUREJOB']._serialized_end=2089
  _globals['_FILEMETADATA']._serialized_start=2091
  _globals['_FILEMETADATA']._serialized_end=2167
  _globals['_FILESTREAMINGMESSAGE']._serialized_start=2170
  _globals['_FILESTREAMINGMESSAGE']._serialized_end=2357
  _globals['_FILETRANSFERCOMPLETEDMESSAGE']._serialized_start=2359
  _globals['_FILETRANSFERCOMPLETEDMESSAGE']._serialized_end=2408
  _globals['_DIRECTORYSTREAMINGMESSAGE']._serialized_start=2411
  _globals['_DIRECTORYSTREAMINGMESSAGE']._serialized_end=2644
  _globals['_DIRECTORYMETADATA']._serialized_start=2646
  _globals['_DIRECTORYMETADATA']._serialized_end=2719
  _globals['_DIRECTORYTRANSFERCOMPLETEDMESSAGE']._serialized_start=2721
  _globals['_DIRECTORYTRANSFERCOMPLETEDMESSAGE']._serialized_end=2756
  _globals['_DIRECTORYSTREAMINGRESPONSE']._serialized_start=2758
  _globals['_DIRECTORYSTREAMINGRESPONSE']._serialized_end=2803
  _globals['_DEVELOPERFUNCTIONLIST']._serialized_start=2805
  _globals['_DEVELOPERFUNCTIONLIST']._serialized_end=2887
  _globals['_DEVELOPERFUNCTIONSOURCE']._serialized_start=2889
  _globals['_DEVELOPERFUNCTIONSOURCE']._serialized_end=2993
  _globals['_DEVELOPERFUNCTIONDEFINITION']._serialized_start=2995
  _globals['_DEVELOPERFUNCTIONDEFINITION']._serialized_end=3117
  _globals['_PARAMETERDEFINITION']._serialized_start=3120
  _globals['_PARAMETERDEFINITION']._serialized_end=3267
  _globals['_NUMERICPARAMETER']._serialized_start=3269
  _globals['_NUMERICPARAMETER']._serialized_end=3385
  _globals['_BOOLPARAMETER']._serialized_start=3387
  _globals['_BOOLPARAMETER']._serialized_end=3443
  _globals['_DEVELOPERFUNCTIONCALLRESULT']._serialized_start=3445
  _globals['_DEVELOPERFUNCTIONCALLRESULT']._serialized_end=3491
  _globals['_BAUTIRODESCRIPTIONMESSAGE']._serialized_start=3493
  _globals['_BAUTIRODESCRIPTIONMESSAGE']._serialized_end=3569
  _globals['_BAUTIROURDFLINK']._serialized_start=3572
  _globals['_BAUTIROURDFLINK']._serialized_end=3742
  _globals['_BAUTIROURDFVISUALMESH']._serialized_start=3744
  _globals['_BAUTIROURDFVISUALMESH']._serialized_end=3863
  _globals['_BAUTIROVISUALFILEREQUEST']._serialized_start=3865
  _globals['_BAUTIROVISUALFILEREQUEST']._serialized_end=3910
  _globals['_ROBOTTRANSFORM']._serialized_start=3912
  _globals['_ROBOTTRANSFORM']._serialized_end=3984
  _globals['_UPDATETRANSFORMMESSAGE']._serialized_start=3986
  _globals['_UPDATETRANSFORMMESSAGE']._serialized_end=4094
  _globals['_BASELAYERSERVICE']._serialized_start=4097
  _globals['_BASELAYERSERVICE']._serialized_end=5229
# @@protoc_insertion_point(module_scope)
