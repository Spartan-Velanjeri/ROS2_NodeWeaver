from google.protobuf import empty_pb2 as _empty_pb2
from google.protobuf import timestamp_pb2 as _timestamp_pb2
import ccu_grpc.common_types_pb2 as _common_types_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class BaseLayerState(_message.Message):
    __slots__ = ["can_switch_to_manual_mode", "can_start_mission", "mission_in_progress"]
    CAN_SWITCH_TO_MANUAL_MODE_FIELD_NUMBER: _ClassVar[int]
    CAN_START_MISSION_FIELD_NUMBER: _ClassVar[int]
    MISSION_IN_PROGRESS_FIELD_NUMBER: _ClassVar[int]
    can_switch_to_manual_mode: bool
    can_start_mission: bool
    mission_in_progress: MissionMetaData
    def __init__(self, can_switch_to_manual_mode: bool = ..., can_start_mission: bool = ..., mission_in_progress: _Optional[_Union[MissionMetaData, _Mapping]] = ...) -> None: ...

class MissionIdRequest(_message.Message):
    __slots__ = ["mission_id"]
    MISSION_ID_FIELD_NUMBER: _ClassVar[int]
    mission_id: int
    def __init__(self, mission_id: _Optional[int] = ...) -> None: ...

class StartMissionResponse(_message.Message):
    __slots__ = ["success"]
    SUCCESS_FIELD_NUMBER: _ClassVar[int]
    success: bool
    def __init__(self, success: bool = ...) -> None: ...

class MissionList(_message.Message):
    __slots__ = ["missions", "current_active_mission_id"]
    MISSIONS_FIELD_NUMBER: _ClassVar[int]
    CURRENT_ACTIVE_MISSION_ID_FIELD_NUMBER: _ClassVar[int]
    missions: _containers.RepeatedCompositeFieldContainer[MissionMetaData]
    current_active_mission_id: int
    def __init__(self, missions: _Optional[_Iterable[_Union[MissionMetaData, _Mapping]]] = ..., current_active_mission_id: _Optional[int] = ...) -> None: ...

class MissionMetaData(_message.Message):
    __slots__ = ["id", "name", "predecessor_id", "import_date", "room_plan_md5"]
    ID_FIELD_NUMBER: _ClassVar[int]
    NAME_FIELD_NUMBER: _ClassVar[int]
    PREDECESSOR_ID_FIELD_NUMBER: _ClassVar[int]
    IMPORT_DATE_FIELD_NUMBER: _ClassVar[int]
    ROOM_PLAN_MD5_FIELD_NUMBER: _ClassVar[int]
    id: int
    name: str
    predecessor_id: int
    import_date: _timestamp_pb2.Timestamp
    room_plan_md5: str
    def __init__(self, id: _Optional[int] = ..., name: _Optional[str] = ..., predecessor_id: _Optional[int] = ..., import_date: _Optional[_Union[_timestamp_pb2.Timestamp, _Mapping]] = ..., room_plan_md5: _Optional[str] = ...) -> None: ...

class MissionDataResponse(_message.Message):
    __slots__ = ["node_tree", "jobs", "room_plan_md5"]
    NODE_TREE_FIELD_NUMBER: _ClassVar[int]
    JOBS_FIELD_NUMBER: _ClassVar[int]
    ROOM_PLAN_MD5_FIELD_NUMBER: _ClassVar[int]
    node_tree: _containers.RepeatedCompositeFieldContainer[Node]
    jobs: _containers.RepeatedCompositeFieldContainer[Job]
    room_plan_md5: str
    def __init__(self, node_tree: _Optional[_Iterable[_Union[Node, _Mapping]]] = ..., jobs: _Optional[_Iterable[_Union[Job, _Mapping]]] = ..., room_plan_md5: _Optional[str] = ...) -> None: ...

class Node(_message.Message):
    __slots__ = ["children", "local_transform", "name_unique_per_node_tree", "series", "fastener_template", "parent_node_name_unique"]
    CHILDREN_FIELD_NUMBER: _ClassVar[int]
    LOCAL_TRANSFORM_FIELD_NUMBER: _ClassVar[int]
    NAME_UNIQUE_PER_NODE_TREE_FIELD_NUMBER: _ClassVar[int]
    SERIES_FIELD_NUMBER: _ClassVar[int]
    FASTENER_TEMPLATE_FIELD_NUMBER: _ClassVar[int]
    PARENT_NODE_NAME_UNIQUE_FIELD_NUMBER: _ClassVar[int]
    children: _containers.RepeatedCompositeFieldContainer[Node]
    local_transform: _common_types_pb2.Matrix4x4
    name_unique_per_node_tree: str
    series: str
    fastener_template: FastenerTemplate
    parent_node_name_unique: str
    def __init__(self, children: _Optional[_Iterable[_Union[Node, _Mapping]]] = ..., local_transform: _Optional[_Union[_common_types_pb2.Matrix4x4, _Mapping]] = ..., name_unique_per_node_tree: _Optional[str] = ..., series: _Optional[str] = ..., fastener_template: _Optional[_Union[FastenerTemplate, _Mapping]] = ..., parent_node_name_unique: _Optional[str] = ...) -> None: ...

class FastenerTemplate(_message.Message):
    __slots__ = ["drill_masks", "name_unique_per_node_tree", "type_guid", "parent_node_name_unique"]
    DRILL_MASKS_FIELD_NUMBER: _ClassVar[int]
    NAME_UNIQUE_PER_NODE_TREE_FIELD_NUMBER: _ClassVar[int]
    TYPE_GUID_FIELD_NUMBER: _ClassVar[int]
    PARENT_NODE_NAME_UNIQUE_FIELD_NUMBER: _ClassVar[int]
    drill_masks: _containers.RepeatedCompositeFieldContainer[DrillMask]
    name_unique_per_node_tree: str
    type_guid: str
    parent_node_name_unique: str
    def __init__(self, drill_masks: _Optional[_Iterable[_Union[DrillMask, _Mapping]]] = ..., name_unique_per_node_tree: _Optional[str] = ..., type_guid: _Optional[str] = ..., parent_node_name_unique: _Optional[str] = ...) -> None: ...

class DrillMask(_message.Message):
    __slots__ = ["local_transform", "name_unique_per_node_tree", "drill_holes", "parent_fastener_template_name_unique"]
    LOCAL_TRANSFORM_FIELD_NUMBER: _ClassVar[int]
    NAME_UNIQUE_PER_NODE_TREE_FIELD_NUMBER: _ClassVar[int]
    DRILL_HOLES_FIELD_NUMBER: _ClassVar[int]
    PARENT_FASTENER_TEMPLATE_NAME_UNIQUE_FIELD_NUMBER: _ClassVar[int]
    local_transform: _common_types_pb2.Matrix4x4
    name_unique_per_node_tree: str
    drill_holes: _containers.RepeatedCompositeFieldContainer[DrillHole]
    parent_fastener_template_name_unique: str
    def __init__(self, local_transform: _Optional[_Union[_common_types_pb2.Matrix4x4, _Mapping]] = ..., name_unique_per_node_tree: _Optional[str] = ..., drill_holes: _Optional[_Iterable[_Union[DrillHole, _Mapping]]] = ..., parent_fastener_template_name_unique: _Optional[str] = ...) -> None: ...

class DrillHole(_message.Message):
    __slots__ = ["local_transform", "name_unique_per_drillmask", "state", "parent_drillmask_name_unique", "d_x", "d_y"]
    class DrillHoleState(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
        UNDEFINED: _ClassVar[DrillHole.DrillHoleState]
        UNDRILLED: _ClassVar[DrillHole.DrillHoleState]
        SCHEDULED: _ClassVar[DrillHole.DrillHoleState]
        FINISHED: _ClassVar[DrillHole.DrillHoleState]
        IGNORE: _ClassVar[DrillHole.DrillHoleState]
        FAILED: _ClassVar[DrillHole.DrillHoleState]
    UNDEFINED: DrillHole.DrillHoleState
    UNDRILLED: DrillHole.DrillHoleState
    SCHEDULED: DrillHole.DrillHoleState
    FINISHED: DrillHole.DrillHoleState
    IGNORE: DrillHole.DrillHoleState
    FAILED: DrillHole.DrillHoleState
    LOCAL_TRANSFORM_FIELD_NUMBER: _ClassVar[int]
    NAME_UNIQUE_PER_DRILLMASK_FIELD_NUMBER: _ClassVar[int]
    STATE_FIELD_NUMBER: _ClassVar[int]
    PARENT_DRILLMASK_NAME_UNIQUE_FIELD_NUMBER: _ClassVar[int]
    D_X_FIELD_NUMBER: _ClassVar[int]
    D_Y_FIELD_NUMBER: _ClassVar[int]
    local_transform: _common_types_pb2.Matrix4x4
    name_unique_per_drillmask: str
    state: DrillHole.DrillHoleState
    parent_drillmask_name_unique: str
    d_x: float
    d_y: float
    def __init__(self, local_transform: _Optional[_Union[_common_types_pb2.Matrix4x4, _Mapping]] = ..., name_unique_per_drillmask: _Optional[str] = ..., state: _Optional[_Union[DrillHole.DrillHoleState, str]] = ..., parent_drillmask_name_unique: _Optional[str] = ..., d_x: _Optional[float] = ..., d_y: _Optional[float] = ...) -> None: ...

class Job(_message.Message):
    __slots__ = ["id_unique", "absolute_transform", "enabled", "drill_job", "move_job", "measure_job"]
    ID_UNIQUE_FIELD_NUMBER: _ClassVar[int]
    ABSOLUTE_TRANSFORM_FIELD_NUMBER: _ClassVar[int]
    ENABLED_FIELD_NUMBER: _ClassVar[int]
    DRILL_JOB_FIELD_NUMBER: _ClassVar[int]
    MOVE_JOB_FIELD_NUMBER: _ClassVar[int]
    MEASURE_JOB_FIELD_NUMBER: _ClassVar[int]
    id_unique: str
    absolute_transform: _common_types_pb2.Matrix4x4
    enabled: bool
    drill_job: DrillJob
    move_job: MoveJob
    measure_job: MeasureJob
    def __init__(self, id_unique: _Optional[str] = ..., absolute_transform: _Optional[_Union[_common_types_pb2.Matrix4x4, _Mapping]] = ..., enabled: bool = ..., drill_job: _Optional[_Union[DrillJob, _Mapping]] = ..., move_job: _Optional[_Union[MoveJob, _Mapping]] = ..., measure_job: _Optional[_Union[MeasureJob, _Mapping]] = ...) -> None: ...

class DrillJob(_message.Message):
    __slots__ = ["drillmask_names"]
    DRILLMASK_NAMES_FIELD_NUMBER: _ClassVar[int]
    drillmask_names: _containers.RepeatedScalarFieldContainer[str]
    def __init__(self, drillmask_names: _Optional[_Iterable[str]] = ...) -> None: ...

class MoveJob(_message.Message):
    __slots__ = []
    def __init__(self) -> None: ...

class MeasureJob(_message.Message):
    __slots__ = []
    def __init__(self) -> None: ...

class FileMetaData(_message.Message):
    __slots__ = ["file_name", "file_size_bytes", "md5_hash"]
    FILE_NAME_FIELD_NUMBER: _ClassVar[int]
    FILE_SIZE_BYTES_FIELD_NUMBER: _ClassVar[int]
    MD5_HASH_FIELD_NUMBER: _ClassVar[int]
    file_name: str
    file_size_bytes: int
    md5_hash: str
    def __init__(self, file_name: _Optional[str] = ..., file_size_bytes: _Optional[int] = ..., md5_hash: _Optional[str] = ...) -> None: ...

class FileStreamingMessage(_message.Message):
    __slots__ = ["file_meta_data", "file_chunk_data", "file_completed"]
    FILE_META_DATA_FIELD_NUMBER: _ClassVar[int]
    FILE_CHUNK_DATA_FIELD_NUMBER: _ClassVar[int]
    FILE_COMPLETED_FIELD_NUMBER: _ClassVar[int]
    file_meta_data: FileMetaData
    file_chunk_data: bytes
    file_completed: FileTransferCompletedMessage
    def __init__(self, file_meta_data: _Optional[_Union[FileMetaData, _Mapping]] = ..., file_chunk_data: _Optional[bytes] = ..., file_completed: _Optional[_Union[FileTransferCompletedMessage, _Mapping]] = ...) -> None: ...

class FileTransferCompletedMessage(_message.Message):
    __slots__ = ["file_name"]
    FILE_NAME_FIELD_NUMBER: _ClassVar[int]
    file_name: str
    def __init__(self, file_name: _Optional[str] = ...) -> None: ...

class DirectoryStreamingMessage(_message.Message):
    __slots__ = ["dir_meta_data", "file_data", "dir_completed"]
    DIR_META_DATA_FIELD_NUMBER: _ClassVar[int]
    FILE_DATA_FIELD_NUMBER: _ClassVar[int]
    DIR_COMPLETED_FIELD_NUMBER: _ClassVar[int]
    dir_meta_data: DirectoryMetaData
    file_data: FileStreamingMessage
    dir_completed: DirectoryTransferCompletedMessage
    def __init__(self, dir_meta_data: _Optional[_Union[DirectoryMetaData, _Mapping]] = ..., file_data: _Optional[_Union[FileStreamingMessage, _Mapping]] = ..., dir_completed: _Optional[_Union[DirectoryTransferCompletedMessage, _Mapping]] = ...) -> None: ...

class DirectoryMetaData(_message.Message):
    __slots__ = ["directory_name", "directory_file_count"]
    DIRECTORY_NAME_FIELD_NUMBER: _ClassVar[int]
    DIRECTORY_FILE_COUNT_FIELD_NUMBER: _ClassVar[int]
    directory_name: str
    directory_file_count: int
    def __init__(self, directory_name: _Optional[str] = ..., directory_file_count: _Optional[int] = ...) -> None: ...

class DirectoryTransferCompletedMessage(_message.Message):
    __slots__ = []
    def __init__(self) -> None: ...

class DirectoryStreamingResponse(_message.Message):
    __slots__ = ["success"]
    SUCCESS_FIELD_NUMBER: _ClassVar[int]
    success: bool
    def __init__(self, success: bool = ...) -> None: ...

class DeveloperFunctionList(_message.Message):
    __slots__ = ["sources"]
    SOURCES_FIELD_NUMBER: _ClassVar[int]
    sources: _containers.RepeatedCompositeFieldContainer[DeveloperFunctionSource]
    def __init__(self, sources: _Optional[_Iterable[_Union[DeveloperFunctionSource, _Mapping]]] = ...) -> None: ...

class DeveloperFunctionSource(_message.Message):
    __slots__ = ["name", "functions"]
    NAME_FIELD_NUMBER: _ClassVar[int]
    FUNCTIONS_FIELD_NUMBER: _ClassVar[int]
    name: str
    functions: _containers.RepeatedCompositeFieldContainer[DeveloperFunctionDefinition]
    def __init__(self, name: _Optional[str] = ..., functions: _Optional[_Iterable[_Union[DeveloperFunctionDefinition, _Mapping]]] = ...) -> None: ...

class DeveloperFunctionDefinition(_message.Message):
    __slots__ = ["name", "description", "parameters"]
    NAME_FIELD_NUMBER: _ClassVar[int]
    DESCRIPTION_FIELD_NUMBER: _ClassVar[int]
    PARAMETERS_FIELD_NUMBER: _ClassVar[int]
    name: str
    description: str
    parameters: _containers.RepeatedCompositeFieldContainer[ParameterDefinition]
    def __init__(self, name: _Optional[str] = ..., description: _Optional[str] = ..., parameters: _Optional[_Iterable[_Union[ParameterDefinition, _Mapping]]] = ...) -> None: ...

class ParameterDefinition(_message.Message):
    __slots__ = ["param_number", "param_bool"]
    PARAM_NUMBER_FIELD_NUMBER: _ClassVar[int]
    PARAM_BOOL_FIELD_NUMBER: _ClassVar[int]
    param_number: NumericParameter
    param_bool: BoolParameter
    def __init__(self, param_number: _Optional[_Union[NumericParameter, _Mapping]] = ..., param_bool: _Optional[_Union[BoolParameter, _Mapping]] = ...) -> None: ...

class NumericParameter(_message.Message):
    __slots__ = ["param_name", "param_value", "min_value", "max_value", "step_size"]
    PARAM_NAME_FIELD_NUMBER: _ClassVar[int]
    PARAM_VALUE_FIELD_NUMBER: _ClassVar[int]
    MIN_VALUE_FIELD_NUMBER: _ClassVar[int]
    MAX_VALUE_FIELD_NUMBER: _ClassVar[int]
    STEP_SIZE_FIELD_NUMBER: _ClassVar[int]
    param_name: str
    param_value: float
    min_value: float
    max_value: float
    step_size: float
    def __init__(self, param_name: _Optional[str] = ..., param_value: _Optional[float] = ..., min_value: _Optional[float] = ..., max_value: _Optional[float] = ..., step_size: _Optional[float] = ...) -> None: ...

class BoolParameter(_message.Message):
    __slots__ = ["param_name", "param_value"]
    PARAM_NAME_FIELD_NUMBER: _ClassVar[int]
    PARAM_VALUE_FIELD_NUMBER: _ClassVar[int]
    param_name: str
    param_value: bool
    def __init__(self, param_name: _Optional[str] = ..., param_value: bool = ...) -> None: ...

class DeveloperFunctionCallResult(_message.Message):
    __slots__ = ["message"]
    MESSAGE_FIELD_NUMBER: _ClassVar[int]
    message: str
    def __init__(self, message: _Optional[str] = ...) -> None: ...

class BautiroDescriptionMessage(_message.Message):
    __slots__ = ["links"]
    LINKS_FIELD_NUMBER: _ClassVar[int]
    links: _containers.RepeatedCompositeFieldContainer[BautiroUrdfLink]
    def __init__(self, links: _Optional[_Iterable[_Union[BautiroUrdfLink, _Mapping]]] = ...) -> None: ...

class BautiroUrdfLink(_message.Message):
    __slots__ = ["name", "parent", "transform_relative_to_parent", "visuals"]
    NAME_FIELD_NUMBER: _ClassVar[int]
    PARENT_FIELD_NUMBER: _ClassVar[int]
    TRANSFORM_RELATIVE_TO_PARENT_FIELD_NUMBER: _ClassVar[int]
    VISUALS_FIELD_NUMBER: _ClassVar[int]
    name: str
    parent: str
    transform_relative_to_parent: _common_types_pb2.Matrix4x4
    visuals: _containers.RepeatedCompositeFieldContainer[BautiroUrdfVisualMesh]
    def __init__(self, name: _Optional[str] = ..., parent: _Optional[str] = ..., transform_relative_to_parent: _Optional[_Union[_common_types_pb2.Matrix4x4, _Mapping]] = ..., visuals: _Optional[_Iterable[_Union[BautiroUrdfVisualMesh, _Mapping]]] = ...) -> None: ...

class BautiroUrdfVisualMesh(_message.Message):
    __slots__ = ["mesh_file_path", "mesh_file_md5_hash", "offset"]
    MESH_FILE_PATH_FIELD_NUMBER: _ClassVar[int]
    MESH_FILE_MD5_HASH_FIELD_NUMBER: _ClassVar[int]
    OFFSET_FIELD_NUMBER: _ClassVar[int]
    mesh_file_path: str
    mesh_file_md5_hash: str
    offset: _common_types_pb2.Matrix4x4
    def __init__(self, mesh_file_path: _Optional[str] = ..., mesh_file_md5_hash: _Optional[str] = ..., offset: _Optional[_Union[_common_types_pb2.Matrix4x4, _Mapping]] = ...) -> None: ...

class BautiroVisualFileRequest(_message.Message):
    __slots__ = ["file_path"]
    FILE_PATH_FIELD_NUMBER: _ClassVar[int]
    file_path: str
    def __init__(self, file_path: _Optional[str] = ...) -> None: ...

class RobotTransform(_message.Message):
    __slots__ = ["links"]
    LINKS_FIELD_NUMBER: _ClassVar[int]
    links: _containers.RepeatedCompositeFieldContainer[UpdateTransformMessage]
    def __init__(self, links: _Optional[_Iterable[_Union[UpdateTransformMessage, _Mapping]]] = ...) -> None: ...

class UpdateTransformMessage(_message.Message):
    __slots__ = ["child_id", "transform_relative_to_parent"]
    CHILD_ID_FIELD_NUMBER: _ClassVar[int]
    TRANSFORM_RELATIVE_TO_PARENT_FIELD_NUMBER: _ClassVar[int]
    child_id: str
    transform_relative_to_parent: _common_types_pb2.Matrix4x4
    def __init__(self, child_id: _Optional[str] = ..., transform_relative_to_parent: _Optional[_Union[_common_types_pb2.Matrix4x4, _Mapping]] = ...) -> None: ...
