from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Optional as _Optional

DESCRIPTOR: _descriptor.FileDescriptor

class Matrix4x4(_message.Message):
    __slots__ = ["r1c1", "r1c2", "r1c3", "r1c4", "r2c1", "r2c2", "r2c3", "r2c4", "r3c1", "r3c2", "r3c3", "r3c4", "r4c1", "r4c2", "r4c3", "r4c4"]
    R1C1_FIELD_NUMBER: _ClassVar[int]
    R1C2_FIELD_NUMBER: _ClassVar[int]
    R1C3_FIELD_NUMBER: _ClassVar[int]
    R1C4_FIELD_NUMBER: _ClassVar[int]
    R2C1_FIELD_NUMBER: _ClassVar[int]
    R2C2_FIELD_NUMBER: _ClassVar[int]
    R2C3_FIELD_NUMBER: _ClassVar[int]
    R2C4_FIELD_NUMBER: _ClassVar[int]
    R3C1_FIELD_NUMBER: _ClassVar[int]
    R3C2_FIELD_NUMBER: _ClassVar[int]
    R3C3_FIELD_NUMBER: _ClassVar[int]
    R3C4_FIELD_NUMBER: _ClassVar[int]
    R4C1_FIELD_NUMBER: _ClassVar[int]
    R4C2_FIELD_NUMBER: _ClassVar[int]
    R4C3_FIELD_NUMBER: _ClassVar[int]
    R4C4_FIELD_NUMBER: _ClassVar[int]
    r1c1: float
    r1c2: float
    r1c3: float
    r1c4: float
    r2c1: float
    r2c2: float
    r2c3: float
    r2c4: float
    r3c1: float
    r3c2: float
    r3c3: float
    r3c4: float
    r4c1: float
    r4c2: float
    r4c3: float
    r4c4: float
    def __init__(self, r1c1: _Optional[float] = ..., r1c2: _Optional[float] = ..., r1c3: _Optional[float] = ..., r1c4: _Optional[float] = ..., r2c1: _Optional[float] = ..., r2c2: _Optional[float] = ..., r2c3: _Optional[float] = ..., r2c4: _Optional[float] = ..., r3c1: _Optional[float] = ..., r3c2: _Optional[float] = ..., r3c3: _Optional[float] = ..., r3c4: _Optional[float] = ..., r4c1: _Optional[float] = ..., r4c2: _Optional[float] = ..., r4c3: _Optional[float] = ..., r4c4: _Optional[float] = ...) -> None: ...
