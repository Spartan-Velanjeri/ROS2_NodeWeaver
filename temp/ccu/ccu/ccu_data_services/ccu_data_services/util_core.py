# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.


from typing import Tuple
from datetime import datetime
from builtin_interfaces.msg import Time
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from bautiro_ros_interfaces.msg import DrillMask, DrillHole, Marker
from ccu_bautiro.NodeTree import CDrillHole, CDrillMask
from ccu_bautiro.Types import CMarker, CMatrix

from ccu_dataservice.conv import rel2abs, tm2pose


###################################################################################################
# ROS-msg to internal / internal to ROS-msg
###################################################################################################


def msg2pose(pose: Pose) -> Tuple[Tuple, Tuple]:
    """Convert ROS -> internal."""
    p = pose.position.x, pose.position.y, pose.position.z
    o = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
    return p, o


def rel2abs_pose_msg(tm_element) -> Pose:
    """Convert Internal -> ROS."""
    tm_abs = rel2abs(tm_element)
    return tm2pose_msg(tm_abs)


def tm2pose_msg(tm: CMatrix) -> Pose:
    """Convert Internal -> ROS."""
    p, q = tm2pose(tm)
    return Pose(position=Point(x=p[0], y=p[1], z=p[2]),
                orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))


def drillmask2msg(dm: CDrillMask) -> DrillMask:
    """Convert Internal -> ROS."""
    msg = DrillMask(id=dm.id, pks_pose=rel2abs_pose_msg(dm))
    dh: CDrillHole
    for dh in dm.drillHoles:
        msg.drill_holes.append(DrillHole(id=dh.id,
                                         state=dh.state.value,
                                         depth=dh.depth,
                                         diameter=dh.diameter,
                                         pks_pose=rel2abs_pose_msg(dh)))
    return msg


def to_sec(iso_date: str) -> int:
    """Seconds since 1970-02-01 from iso-date-string like `2006-05-04T18:13:51.0`."""
    if iso_date:
        try:
            dt = datetime.strptime(iso_date, "%Y-%m-%dT%H:%M:%S.%f")
            delta = dt - datetime(1970, 1, 1)
            return int(delta.total_seconds())
        except ValueError as e:
            print(f'def to_sec(iso_date: str) -> int: {e}')
    return 0


def marker2msg(m: CMarker) -> Marker:
    """Convert Internal -> ROS."""
    return Marker(marker_id=m.id,
                  code=m.code,
                  parent=m.parent,
                  state=m.state.value,  # ccu_bautiro.Types.MARKER_STATE.value == Marker.msg.state
                  pose=PoseStamped(pose=tm2pose_msg(m.tm),
                                   header=Header(stamp=Time(sec=to_sec(m.time)))
                                   ))
