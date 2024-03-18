# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction
# editing, distribution, as well as in the event of applications for industrial
# property rights.


from geometry_msgs.msg import Pose, Point, Quaternion
from ccu_grpc.manual_mode_service_pb2 import StartDrillMessage
from ccu_grpc.common_types_pb2 import Matrix4x4
from bautiro_ros_interfaces.msg import DrillMask, DrillHole
from hashlib import md5   # noqa: E402

RED = '\033[91m'
GRN = '\033[92m'
YEL = '\033[93m'
BLU = '\033[94m'
MGN = '\033[95m'
CYN = '\033[96m'
WHT = '\033[97m'
NOC = '\033[0m'


class DummyLogger():
    def _color_print(self, txt: str, color_code: str):
        txt = txt if isinstance(txt, str) else repr(txt)
        print(f'{color_code}txt{NOC}')
        return txt

    def info(self, txt: str):
        return self._color_print(txt, NOC)

    def warn(self, txt: str):
        return self._color_print(txt, YEL)

    def error(self, txt: str):
        return self._color_print(txt, RED)

    def fatal(self, txt: str):
        return self._color_print(txt, MGN)

###################################################################################################
#
#   calc_md5
#
###################################################################################################


def calc_md5(file_abs_path) -> str:
    if file_abs_path:
        hash_md5 = md5()
        try:
            with open(file_abs_path, "rb") as f:
                for chunk in iter(lambda: f.read(4096), b""):
                    hash_md5.update(chunk)
            return hash_md5.hexdigest()
        except IOError as e:
            print(f'IOError on opening file {file_abs_path}: {e}')
    return '0' * 32


###################################################################################################
#
#   size
#
###################################################################################################
from os.path import getsize  # noqa: E402


def size(file_abs_path):
    try:
        return getsize(file_abs_path)
    except OSError as e:
        print(f"ERROR: {e}")
        return 0

###################################################################################################
#
#   context.is_active with error-log
#
###################################################################################################


def is_active(context) -> bool:
    is_active: bool = context.is_active()
    if not is_active:
        print("Error: Lost grpg connection: context.is_active() returns False")
    return is_active


###################################################################################################
#
#   stream_file (generic)
#
###################################################################################################
from typing import Generator, Any                                                       # noqa E402
from ccu_grpc.base_layer_service_pb2 import (FileTransferCompletedMessage,              # noqa E402
                                             FileMetaData, FileStreamingMessage)


def stream_file(name: str, abs_path: str, context) -> Generator[FileStreamingMessage, None, Any]:
    if is_active(context):
        yield FileStreamingMessage(file_meta_data=FileMetaData(file_name=name,
                                                               file_size_bytes=size(abs_path),
                                                               md5_hash=calc_md5(abs_path)))
    with open(abs_path, 'rb') as f:
        while is_active(context):
            if not (chunk := f.read(1024 * 10)):
                break
            yield FileStreamingMessage(file_chunk_data=chunk)
    if is_active(context):
        yield FileStreamingMessage(file_completed=FileTransferCompletedMessage(file_name=name))


###################################################################################################
#                                                                                                 #
#  Iterate DirectoryStream and persist payload to according files                                 #
#                                                                                                 #
###################################################################################################
from typing import List                                                                 # noqa E402
from os import makedirs                                                                 # noqa E402
from os.path import join                                                                # noqa E402
from io import BufferedWriter                                                           # noqa E402
from shutil import rmtree                                                               # noqa E402
from ccu_grpc.base_layer_service_pb2 import (DirectoryStreamingMessage,                 # noqa E402
                                             FileStreamingMessage)


def receive_directory_stream(tmp_dir: str,
                             dir_stream: List[DirectoryStreamingMessage],
                             context) -> int:
    """Return `-1` on errors -else- number of Files received from `0 ... n`."""

    rmtree(tmp_dir, ignore_errors=True)
    makedirs(tmp_dir, exist_ok=True)

    if not is_active(context):
        return -1
    for first in dir_stream:
        if 'dir_meta_data' != first.WhichOneof('data'):
            print("Error: Very First Message must be of type 'dir_meta_data'")
            break
        else:
            print(f' Start transfer - directory_name: {first.dir_meta_data.directory_name}\n' +
                  f'                      file_count: {first.dir_meta_data.directory_file_count}')
            number_files = _receive_file_stream(tmp_dir, dir_stream, context)
            _warn_wrong_file_count(number_files, first.dir_meta_data.directory_file_count)
            return number_files
    return -1


def _receive_file_stream(tmp_dir: str,
                         dir_stream: List[DirectoryStreamingMessage],
                         context) -> int:
    """Return number files received."""
    if not context.is_active():
        return -1
    files = 0
    f = None
    for dir_msg in dir_stream:
        if 'file_data' == dir_msg.WhichOneof('data'):
            fsm: FileStreamingMessage = dir_msg.file_data
            if 'file_meta_data' != fsm.WhichOneof('data'):
                print("Error: Message at this point must be of type 'file_meta_data'")
                break
            else:
                print(f' Start file transfer of : {fsm.file_meta_data.file_name}\n' +
                      f'             size(bytes): {fsm.file_meta_data.file_size_bytes}\n' +
                      f'                     md5: {fsm.file_meta_data.md5_hash}')
                f = open(join(tmp_dir, fsm.file_meta_data.file_name), 'wb')
                if _receive_chunk_stream(f, dir_stream, context):
                    files += 1
                else:
                    break
        elif 'dir_completed' == dir_msg.WhichOneof('data'):
            if f and not f.closed:
                f.close()
            return files
        else:
            print("Error: Subsequent messages at this point must be of type " +
                  "'file_data' or 'dir_completed'")
            break
    print("DEBUG: directory_stream not ended with 'dir_completed' message !!!")
    if f and not f.closed:
        f.close()
    return files


def _receive_chunk_stream(f: BufferedWriter,
                          dir_stream: List[DirectoryStreamingMessage],
                          context) -> bool:

    def closing_false() -> bool:
        f.close()
        return False

    if not is_active(context):
        return closing_false()

    chunks = 0
    for dir_msg in dir_stream:
        if 'file_data' != dir_msg.WhichOneof('data'):
            print("Error: Subsequent messages at this point must be of type 'file_data'")
            break
        fsm: FileStreamingMessage = dir_msg.file_data
        if 'file_chunk_data' == fsm.WhichOneof('data'):
            print(f'[{chunks}:{len(fsm.file_chunk_data)}b]', end='')
            f.write(fsm.file_chunk_data)
            chunks += 1
        elif 'file_completed' == fsm.WhichOneof('data'):
            print(f'\ncompleted file: {fsm.file_completed.file_name}')
            f.close()
            return True
        else:
            print("Error: Subsequent messages at this point must be of type " +
                  "'completed file' or 'file_chunk_data'")
        if not is_active(context):
            break
    f.close()
    return False


def _warn_wrong_file_count(files, expected) -> None:
    if files > expected:
        print(f'Number of files larger  than expected  {files} > {expected}   [ACCEPTED]')
    elif files < expected:
        print(f'Number of files smaller than expected  {files} < {expected}   [ACCEPTED]')


###################################################################################################
#
#   roll pitch yaw to homogenous matrix
#
###################################################################################################
import math                                                                             # noqa E402
from typing import Tuple                                                                # noqa E402
from ccu_grpc.common_types_pb2 import Matrix4x4                                         # noqa E402


def rpy_to_matrix4x4(xyz_rpy: Tuple[Tuple[str, str, str], Tuple[str, str, str]]) -> Matrix4x4:
    m4 = _rpy_to_4x4(xyz_rpy)
    return m4_to_matrix4x4(m4)


def m4_to_matrix4x4(m4):
    return Matrix4x4(r1c1=m4[0][0], r1c2=m4[0][1], r1c3=m4[0][2], r1c4=m4[0][3],
                     r2c1=m4[1][0], r2c2=m4[1][1], r2c3=m4[1][2], r2c4=m4[1][3],
                     r3c1=m4[2][0], r3c2=m4[2][1], r3c3=m4[2][2], r3c4=m4[2][3],
                     r4c1=m4[3][0], r4c2=m4[3][1], r4c3=m4[3][2], r4c4=m4[3][3])


def _rpy_to_4x4(xyz_rpy: Tuple[Tuple[str, str, str], Tuple[str, str, str]]):
    x = y = z = rol = pit = yaw = 0.0
    if xyz_rpy:
        if xyz := xyz_rpy[0]:
            x, y, z = map(float, xyz)
        if rpy := xyz_rpy[1]:
            rol, pit, yaw = map(float, rpy)

    cr, cp, cy = math.cos(rol), math.cos(pit), math.cos(yaw)
    sr, sp, sy = math.sin(rol), math.sin(pit), math.sin(yaw)

    r = [[cy*cp,   cy*sp*sr-sy*cr,  cy*sp*cr+sy*sr],
         [sy*cp,   sy*sp*sr+cy*cr,  sy*sp*cr-cy*sr],
         [-sp,            cp*sr,            cp*cr]]
    return [[r[0][0], r[0][1], r[0][2], x],
            [r[1][0], r[1][1], r[1][2], y],
            [r[2][0], r[2][1], r[2][2], z],
            [0,             0,       0, 1]]


###################################################################################################
#   numpy way
###################################################################################################
import numpy as np                                                                      # noqa E402


def _rpy_to_4x4_np(xyz_rpy: Tuple[Tuple[str, str, str], Tuple[str, str, str]]) -> np.ndarray:
    x = y = z = rol = pit = yaw = 0.0
    if xyz_rpy:
        if xyz := xyz_rpy[0]:
            x, y, z = map(float, xyz)
        if rpy := xyz_rpy[1]:
            rol, pit, yaw = map(float, rpy)

    # Calculating trigonometric values for roll, pitch, yaw
    cr, sr = math.cos(rol), math.sin(rol)
    cp, sp = math.cos(pit), math.sin(pit)
    cy, sy = math.cos(yaw), math.sin(yaw)

    # Creating rotation matrices
    rot_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    rot_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rot_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])

    # Combining the rotation matrices
    rot = np.dot(rot_z, np.dot(rot_y, rot_x))

    # Creating a translation matrix
    translation = np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])

    # Generating the 4x4 transformation matrix (rotation + translation)
    return np.dot(translation, np.vstack((np.hstack((rot, np.zeros((3, 1)))), [0, 0, 0, 1])),)


###################################################################################################
#
#   geometry_msgs/Transform to Matrix4x4
#
###################################################################################################

from geometry_msgs.msg import Transform, Quaternion, Vector3                            # noqa E402
from ccu_grpc.common_types_pb2 import Matrix4x4                                         # noqa E402

# SEE     src/bautiro_common/BIMConnector_xsd/DataTypes/DataTypes-1.0/DataTypes-1.0.xsd
# LINE 29:     <xs:complexType name="Matrix">

# SEE     src/ccu_data_services/ccu_dataservice/src/ccu_dataservice/loader.py
# LINE 57:     def convert_tm()


def tf_to_matrix4x4(transform: Transform) -> Matrix4x4:
    return vq_to_matrix4x4(translation=transform.translation,
                           rotation=transform.rotation)


def vq_to_matrix4x4(transl: Vector3, rotation: Quaternion) -> Matrix4x4:
    m3 = ros_quaternion_to_matrix_3x3(rotation)
    return Matrix4x4(r1c1=m3[0][0], r1c2=m3[0][1], r1c3=m3[0][2], r1c4=0.0,
                     r2c1=m3[1][0], r2c2=m3[1][1], r2c3=m3[1][2], r2c4=0.0,
                     r3c1=m3[2][0], r3c2=m3[2][1], r3c3=m3[2][2], r3c4=0.0,
                     r4c1=transl.x, r4c2=transl.y, r4c3=transl.z, r4c4=1)


def ros_quaternion_to_matrix_3x3(quaternion: Quaternion):
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w,

    # [ row1,
    #   row2,
    #   row3 ]
    return [[1 - 2*y**2 - 2*z**2,     2*x*y - 2*z*w,       2*x*z + 2*y*w],
            [2*x*y + 2*z*w,           1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w,           2*y*z + 2*x*w,       1 - 2*x**2 - 2*y**2]]

###################################################################################################
#
#   used in fpm_skill
#   - drillholes2drillmask
#   - matrix4x4_to_pose
#
###################################################################################################


def matrix4x4_to_pose(m: Matrix4x4) -> Pose:
    return Pose(position=Point(x=m.r4c1, y=m.r4c2, z=m.r4c3),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))  # FIXME Rotations not DONE !


def drill_holes_to_drill_mask(drill_holes: List[StartDrillMessage.ManualDrillHole]
                              ) -> List[DrillMask]:
    dm = DrillMask(pks_pose=Pose(position=Point(x=0.0, y=0.0, z=0.0),
                                 orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),  # FIXME ?
                   drill_holes=[_dh_proto_to_ros(dh) for dh in drill_holes])
    return [dm]


def _dh_proto_to_ros(dh: StartDrillMessage.ManualDrillHole) -> DrillHole:
    return DrillHole(id=dh.name,
                     depth=dh.depth_meter,        # e.g. 0.040 ...  expected for 40mm ∅ 8mm
                     diameter=dh.diameter_meter,  # e.g. 0.008 ...  FIXME  is this expected
                     pks_pose=matrix4x4_to_pose(dh.local_transform),
                     state=DrillHole.UNDRILLED)


###################################################################################################
#
#   used in ccu_hcu_abstraction
#   developer_functions
#
###################################################################################################

def dup_locate(string_list: List[str]):
    seen = set()
    duplicates = set()
    for string in string_list:
        if string in seen:
            duplicates.add(string)
        else:
            seen.add(string)
    return duplicates

###################################################################################################
#
#   EOF
#
###################################################################################################
