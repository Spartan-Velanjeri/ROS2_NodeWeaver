# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.


from typing import List
from datetime import datetime

from google.protobuf.timestamp_pb2 import Timestamp

from ccu_grpc.common_types_pb2 import Matrix4x4

from ccu_grpc.base_layer_service_pb2 import (MissionMetaData, MissionDataResponse,
                                             Job, DrillJob, MeasureJob, MoveJob,
                                             Node, FastenerTemplate, DrillMask, DrillHole)

from ccu_bautiro.Types import CMatrix
from ccu_bautiro.MetaData import CMissionMD
from ccu_bautiro.NodeTree import CNode, CFastener, CDrillMask, CDrillHole
from ccu_bautiro.Jobs import CJob,  CMeasureJob
from ccu_dataservice.core import DSContext

from ccu_dataservice.eutil import is_same_classname


def cmatrix_to_matrix4x4(tm: CMatrix):
    return Matrix4x4(
        r1c1=tm.v1x,
        r1c2=tm.v1y,
        r1c3=tm.v1z,
        r1c4=0,

        r2c1=tm.v2x,
        r2c2=tm.v2y,
        r2c3=tm.v2z,
        r2c4=0,

        r3c1=tm.v3x,
        r3c2=tm.v3y,
        r3c3=tm.v3z,
        r3c4=0,

        r4c1=tm.v0x,
        r4c2=tm.v0y,
        r4c3=tm.v0z,
        r4c4=1,
    )


###################################################################################################
#                                                                                                 #
#  get_mission_list                                                                               #
#                                                                                                 #
###################################################################################################


def create_missionmetadata(md: CMissionMD = None, md5: str = None) -> MissionMetaData:
    if not md:
        return MissionMetaData()
    ts = Timestamp()
    ts.FromDatetime(datetime.strptime(md.dateTimeCreation, '%Y-%m-%dT%H:%M:%S.%f'))
    return MissionMetaData(id=int(md.index),
                           import_date=ts,
                           name=md.name,
                           predecessor_id=md.predecessorIDs[0] if md.predecessorIDs else -1,
                           room_plan_md5=md5 or ('0' * 32))


###################################################################################################
#                                                                                                 #
# get_mission_data_response                                                                       #
#                                                                                                 #
###################################################################################################


def get_mission_data(ctx: DSContext, mission_index: int) -> MissionDataResponse:
    """Return gRPC message with node-tree of requested missions workplan-nodetree or None."""

    # Magic -7 set in HALC when pulling every 10s for get_mission_data
    if -7 == mission_index:
        mission_index = ctx.mim.active_mission_index

    if ctx.mim.is_valid_index(mission_index):
        mi_md = ctx.mim.get_metadata(mission_index)
        wp_md = ctx.wpm.get_metadata(mi_md.workPlanIndex)
        rp_md = ctx.rpm.get_metadata(wp_md.roomPlanIndex)
        md5 = rp_md.calculated_MD5_32bitHex if rp_md else None

        if wp := ctx.wpm.get_data_file_content(mi_md.workPlanIndex):
            jobs = ctx.mim.get_data_file_content(mission_index).jobs
            xxxx = MissionDataResponse(node_tree=[_create_node(n) for n in wp.nodeTree],
                                       jobs=_create_jobs(jobs, wp.jobs),
                                       room_plan_md5=md5)
            return xxxx

    print(f'Mission not exists : {mission_index}')
    return MissionDataResponse()


def _create_node(node: CNode, parent_node_id: str = None) -> Node:
    xxxx = Node(
        children=[_create_node(child, node.id) for child in node.children],
        local_transform=cmatrix_to_matrix4x4(node.tm),
        name_unique_per_node_tree=node.id,
        series=node.series,
        # fastener_template=_create_fastener(node),
        parent_node_name_unique=parent_node_id or None)  # Optional in .proto--> None is mandatory
    return xxxx


def _create_fastener(n: CNode) -> FastenerTemplate:
    if n.fastener is None:
        return None
    f: CFastener = n.fastener
    return FastenerTemplate(
        drill_masks=[_create_drillmask(dm, f) for dm in f.drillMasks],
        name_unique_per_node_tree=f.id,
        type_guid=f.guid,
        parent_node_name_unique=n.id)  # NOT Optional --> '' correct for String


def _create_drillmask(dm: CDrillMask, f: CFastener) -> DrillMask:
    return DrillMask(local_transform=cmatrix_to_matrix4x4(dm.tm),
                     name_unique_per_node_tree=dm.id,
                     drill_holes=[_create_drillhole(dh) for dh in dm.drillHoles],
                     parent_fastener_template_name_unique=f.id)


def _create_drillhole(dh: CDrillHole, drillmask_name: str = None) -> DrillHole:
    return DrillHole(local_transform=cmatrix_to_matrix4x4(dh.tm),
                     name_unique_per_drillmask=dh.id,
                     state=dh.state.value,
                     parent_drillmask_name_unique=drillmask_name or None,  # Optional --> None
                     d_x=dh.dx,
                     d_y=dh.dy)


def _create_jobs(mission_jobs: List[CJob], all_jobs: List[CJob]) -> List[Job]:
    result: List[Job] = []
    _done = []

    # first add scheduled Jobs
    for cj in mission_jobs:
        result.append(_create_job(cj, True))
        _done.append(cj.id)

    # then remaining jobs (the ones not scheduled)
    for cj in all_jobs:
        if (cj.id not in _done):
            result.append(_create_job(cj, False))
    return result


def _create_job(cj: CJob, enabled) -> Job:
    if hasattr(cj, 'drillMasks'):
        return Job(id_unique=cj.id,
                   absolute_transform=cmatrix_to_matrix4x4(cj.tm),
                   enabled=enabled,
                   drill_job=DrillJob(drillmask_names=[dm.id for dm in cj.drillMasks]))
    elif is_same_classname(cj, CMeasureJob):
        return Job(id_unique=cj.id,
                   absolute_transform=cmatrix_to_matrix4x4(cj.tm),
                   enabled=enabled,
                   measure_job=MeasureJob())
    else:  # elif is_same_classname(cj, CMoveJob):
        return Job(id_unique=cj.id,
                   absolute_transform=cmatrix_to_matrix4x4(cj.tm),
                   enabled=enabled,
                   move_job=MoveJob())
