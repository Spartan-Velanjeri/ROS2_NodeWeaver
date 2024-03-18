from google.protobuf import empty_pb2 as _empty_pb2
import ccu_grpc.common_types_pb2 as _common_types_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class ControlState(_message.Message):
    __slots__ = ["can_drive_rpm", "can_raise_lift", "can_lower_lift", "can_move_arm"]
    CAN_DRIVE_RPM_FIELD_NUMBER: _ClassVar[int]
    CAN_RAISE_LIFT_FIELD_NUMBER: _ClassVar[int]
    CAN_LOWER_LIFT_FIELD_NUMBER: _ClassVar[int]
    CAN_MOVE_ARM_FIELD_NUMBER: _ClassVar[int]
    can_drive_rpm: bool
    can_raise_lift: bool
    can_lower_lift: bool
    can_move_arm: bool
    def __init__(self, can_drive_rpm: bool = ..., can_raise_lift: bool = ..., can_lower_lift: bool = ..., can_move_arm: bool = ...) -> None: ...

class DriveRpmControlMessage(_message.Message):
    __slots__ = ["forward_backward_axis", "left_right_axis", "speed_mult"]
    FORWARD_BACKWARD_AXIS_FIELD_NUMBER: _ClassVar[int]
    LEFT_RIGHT_AXIS_FIELD_NUMBER: _ClassVar[int]
    SPEED_MULT_FIELD_NUMBER: _ClassVar[int]
    forward_backward_axis: float
    left_right_axis: float
    speed_mult: float
    def __init__(self, forward_backward_axis: _Optional[float] = ..., left_right_axis: _Optional[float] = ..., speed_mult: _Optional[float] = ...) -> None: ...

class DriveArmControlMessage(_message.Message):
    __slots__ = ["direction", "speed_mult"]
    class Direction(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
        LEFT: _ClassVar[DriveArmControlMessage.Direction]
        RIGHT: _ClassVar[DriveArmControlMessage.Direction]
        FORWARD: _ClassVar[DriveArmControlMessage.Direction]
        BACKWARD: _ClassVar[DriveArmControlMessage.Direction]
    LEFT: DriveArmControlMessage.Direction
    RIGHT: DriveArmControlMessage.Direction
    FORWARD: DriveArmControlMessage.Direction
    BACKWARD: DriveArmControlMessage.Direction
    DIRECTION_FIELD_NUMBER: _ClassVar[int]
    SPEED_MULT_FIELD_NUMBER: _ClassVar[int]
    direction: DriveArmControlMessage.Direction
    speed_mult: float
    def __init__(self, direction: _Optional[_Union[DriveArmControlMessage.Direction, str]] = ..., speed_mult: _Optional[float] = ...) -> None: ...

class SetStateMessage(_message.Message):
    __slots__ = ["target_state"]
    class TargetState(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
        UNUSED: _ClassVar[SetStateMessage.TargetState]
        DRIVING: _ClassVar[SetStateMessage.TargetState]
        FINE_POSITIONING: _ClassVar[SetStateMessage.TargetState]
    UNUSED: SetStateMessage.TargetState
    DRIVING: SetStateMessage.TargetState
    FINE_POSITIONING: SetStateMessage.TargetState
    TARGET_STATE_FIELD_NUMBER: _ClassVar[int]
    target_state: SetStateMessage.TargetState
    def __init__(self, target_state: _Optional[_Union[SetStateMessage.TargetState, str]] = ...) -> None: ...

class SetStateMessageResponse(_message.Message):
    __slots__ = ["progress", "finished"]
    class SetStateProgressMessage(_message.Message):
        __slots__ = ["progress"]
        PROGRESS_FIELD_NUMBER: _ClassVar[int]
        progress: float
        def __init__(self, progress: _Optional[float] = ...) -> None: ...
    class SetStateFinishedMessage(_message.Message):
        __slots__ = ["success"]
        SUCCESS_FIELD_NUMBER: _ClassVar[int]
        success: bool
        def __init__(self, success: bool = ...) -> None: ...
    PROGRESS_FIELD_NUMBER: _ClassVar[int]
    FINISHED_FIELD_NUMBER: _ClassVar[int]
    progress: SetStateMessageResponse.SetStateProgressMessage
    finished: SetStateMessageResponse.SetStateFinishedMessage
    def __init__(self, progress: _Optional[_Union[SetStateMessageResponse.SetStateProgressMessage, _Mapping]] = ..., finished: _Optional[_Union[SetStateMessageResponse.SetStateFinishedMessage, _Mapping]] = ...) -> None: ...

class StartDrillMessage(_message.Message):
    __slots__ = ["drill_holes"]
    class ManualDrillHole(_message.Message):
        __slots__ = ["local_transform", "name", "depth_meter", "diameter_meter"]
        LOCAL_TRANSFORM_FIELD_NUMBER: _ClassVar[int]
        NAME_FIELD_NUMBER: _ClassVar[int]
        DEPTH_METER_FIELD_NUMBER: _ClassVar[int]
        DIAMETER_METER_FIELD_NUMBER: _ClassVar[int]
        local_transform: _common_types_pb2.Matrix4x4
        name: str
        depth_meter: float
        diameter_meter: float
        def __init__(self, local_transform: _Optional[_Union[_common_types_pb2.Matrix4x4, _Mapping]] = ..., name: _Optional[str] = ..., depth_meter: _Optional[float] = ..., diameter_meter: _Optional[float] = ...) -> None: ...
    DRILL_HOLES_FIELD_NUMBER: _ClassVar[int]
    drill_holes: _containers.RepeatedCompositeFieldContainer[StartDrillMessage.ManualDrillHole]
    def __init__(self, drill_holes: _Optional[_Iterable[_Union[StartDrillMessage.ManualDrillHole, _Mapping]]] = ...) -> None: ...

class StartDrillResponse(_message.Message):
    __slots__ = ["progress", "finished"]
    class DrillFinishedMessage(_message.Message):
        __slots__ = ["success"]
        SUCCESS_FIELD_NUMBER: _ClassVar[int]
        success: bool
        def __init__(self, success: bool = ...) -> None: ...
    PROGRESS_FIELD_NUMBER: _ClassVar[int]
    FINISHED_FIELD_NUMBER: _ClassVar[int]
    progress: float
    finished: StartDrillResponse.DrillFinishedMessage
    def __init__(self, progress: _Optional[float] = ..., finished: _Optional[_Union[StartDrillResponse.DrillFinishedMessage, _Mapping]] = ...) -> None: ...
