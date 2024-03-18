# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

###################################################################################################
#                                                                                                 #
# DRY: Proto-msg de/en-coding                                                                     #
#                                                                                                 #
###################################################################################################

from typing import Type, TypeVar
from base64 import b64decode, b64encode


T = TypeVar('T')


def to_str(proto_msg) -> str:
    """Return `content` of any proto-msg as string."""
    assert proto_msg is not None
    b: bytes = proto_msg.SerializePartialToString(deterministic=True)
    x = b64encode(b)
    return x.decode('unicode_escape')


def inst(typ: Type[T], content: str) -> T:
    """Instantiate proto-msg of type `typ` and fill with `content`."""
    proto_msg = typ()
    proto_msg.ParseFromString(_to_bytes(content))
    return proto_msg


def _to_bytes(s: str) -> bytes:
    return b64decode(s)
