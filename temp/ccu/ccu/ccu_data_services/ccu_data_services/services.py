# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction
# editing, distribution, as well as in the event of applications for industrial
# property rights.


from typing import List
from rclpy.node import Node
from std_srvs.srv import Trigger
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup as CbG
from bautiro_developer_functions.core.node_delegator import NodeDelegator
from bautiro_ros_interfaces.msg import ClusterPosition, DrillHoleFeedback, NavigationResult
from bautiro_ros_interfaces.srv import (AddMissionEvent, GetClusterPosition,
                                        GetMarkersCluster, GetDrillHolesCluster)
from ccu_interfaces.msg import ProtoString
from ccu_interfaces.srv import ProtoInRos, GenericPrim

from ccu_bautiro.MetaData.MetaData import CMissionMD
from ccu_grpc.base_layer_service_pb2 import MissionList, MissionMetaData, MissionIdRequest
from ccu_grpc.util import inst, to_str
from ccu_dataservice.core import DSContext, CoreDS, HcuDS
from ccu_dataservice.manager import DrillHoleResult, NavResult

from ccu_data_services.util_core import tm2pose_msg, drillmask2msg, marker2msg, msg2pose
from ccu_data_services.util_halc import get_mission_data, create_missionmetadata


def bind_services(node: Node, ctx: DSContext, missions_dir: str):
    CoreServices(node, ctx, missions_dir)
    HalcServices(node, ctx)


class CoreServices(NodeDelegator):
    """Offer `(data)services` to `BAUTIRO-core`."""

    def __init__(self, node: Node, ctx: DSContext, missions_dir: str):
        super().__init__(node)
        self.ds = CoreDS(ctx, missions_dir)
        node.create_service(GetClusterPosition,
                            'all_cluster_positions',
                            self.all_cluster_positions,
                            callback_group=CbG())
        node.create_service(GetClusterPosition,
                            GetClusterPosition.Request.NAME,
                            self.get_cluster_position,
                            callback_group=CbG())
        node.create_service(GetMarkersCluster,
                            GetMarkersCluster.Request.NAME,
                            self.get_markers_cluster,
                            callback_group=CbG())
        node.create_service(GetDrillHolesCluster,
                            GetDrillHolesCluster.Request.NAME,
                            self.get_drill_holes_cluster,
                            callback_group=CbG())
        node.create_service(AddMissionEvent,
                            AddMissionEvent.Request.NAME,
                            self.add_mission_event,
                            callback_group=CbG())

    def all_cluster_positions(self, q: GetClusterPosition.Request, r: GetClusterPosition.Response):
        for cj in self.ds.all_cluster_jobs():
            cp = ClusterPosition(cluster_id=cj.id, pks_pose=tm2pose_msg(cj.tm))
            r.cluster_positions.append(cp)
        return r

    def get_cluster_position(self, q: GetClusterPosition.Request, r: GetClusterPosition.Response):
        """Return list of the next ClusterPositions. Returns empty list when no job left to do."""
        self.log('call begin : get_cluster_position()')
        for cj in self.ds.get_next_drill_jobs(q.number_of_next):
            x = cj.id
            cp = ClusterPosition(cluster_id=x, pks_pose=tm2pose_msg(cj.tm))
            r.cluster_positions.append(cp)
        self.log('call   end : get_cluster_position()')
        return r

    def get_drill_holes_cluster(self, q, r: GetDrillHolesCluster.Response):
        """Return DrillMasks (with nested DrillHoles) of the current/next Cluster(-Job)."""
        self.log('call begin : get_drill_holes_cluster()')
        if job := self.ds.get_next_drilljob():
            for dm in job.drillMasks:
                r.drill_masks.append(drillmask2msg(dm))
        self.log('call   end : get_drill_holes_cluster()')
        return r

    def get_markers_cluster(self, q, r: GetMarkersCluster.Response):
        """Return Markers of current/next Cluster(-Job). Empty when no Job left."""
        self.log('call begin : get_markers_cluster()')
        if job := self.ds.get_next_drilljob():
            for m in job.marker:
                r.markers.append(marker2msg(m))
        self.log('call   end : get_markers_cluster()')
        return r

    def add_mission_event(self, q: AddMissionEvent.Request, r: AddMissionEvent.Response):
        """Update Navigation/Drill-Result in (active) WorkPlan."""
        self.log('call begin : add_mission_event()')
        if q.navigation_result.cluster_id:
            r.r_code = self._handle_navigation_result(q.navigation_result)
        if q.drill_results:
            r.r_code = self._handle_drill_results(q.drill_results)
        self.log('call   end : add_mission_event()')
        return r

    def _handle_navigation_result(self, nr: NavigationResult):
        self.ds.add_mission_event(nav_result=NavResult(nr.cluster_id, nr.nav_goal_reached))
        return AddMissionEvent.Response.OKAY

    def _handle_drill_results(self, drill_hole_feedbacks: List[DrillHoleFeedback]):
        de = []
        for f in drill_hole_feedbacks:
            de.append(DrillHoleResult(id=f.id, state_val=f.state,
                                      pose_start=msg2pose(f.abs_pose_pks_start),
                                      pose_finish=msg2pose(f.abs_pose_pks_finish)))
        self.ds.add_mission_event(drill_event=de)
        return AddMissionEvent.Response.OKAY


class HalcServices(NodeDelegator):
    """"Offer `(data)services` to `HCU`."""

    def __init__(self, node: Node, ctx: DSContext):
        super().__init__(node)
        self.ds = HcuDS(ctx)

        self.pub = node.create_publisher(ProtoString, 'current_active_mission_metadata', 10)
        self._old_index = -2
        node.create_timer(1.0, self._pub_curr_act_mi)

        node.create_service(ProtoInRos,
                            'get_mission_data',
                            self.get_mission_data,
                            callback_group=CbG())
        node.create_service(ProtoInRos,
                            'get_mission_list',
                            self.get_mission_list,
                            callback_group=CbG())
        node.create_service(GenericPrim,
                            'set_active_mission',
                            self.set_active_mission,
                            callback_group=CbG())
        node.create_service(GenericPrim,
                            'import_mission',
                            self.import_mission,
                            callback_group=CbG())
        node.create_service(GenericPrim,
                            'get_obj_file',
                            self.get_obj_file,
                            callback_group=CbG())
        node.create_service(Trigger,
                            'unset_active_mission',
                            self.unset_active_mission,
                            callback_group=CbG())

    def _pub_curr_act_mi(self):
        """Publish `current_active_mission_metadata` every second."""

        if mi_md := self.ds.get_mission():
            mmd = self.conv_md(mi_md)
        else:
            mmd = MissionMetaData(id=-1)
        self.pub.publish(ProtoString(proto=to_str(mmd)))
        if self._old_index != mmd.id:
            self.log(f'Publish (1x per /s) MissionMetaData id : {mmd.id}')
        self._old_index = mmd.id

    def conv_md(self, mi_md: CMissionMD) -> MissionMetaData:
        md5_tp = self.ds.get_md5_tp(mi_md.index) if mi_md else None
        md5_calc = self.ds.get_md5_calc(mi_md.index) if mi_md else None
        return create_missionmetadata(mi_md, md5_tp or md5_calc)

    def get_mission_data(self, q: ProtoInRos.Request, r: ProtoInRos.Response):
        """Return `MissionData` of requested `mission_id` (as protobuf)."""
        index = inst(MissionIdRequest, q.proto).mission_id
        self.log(f'call begin : get_mission_data({index})')
        r.proto = to_str(get_mission_data(self.ds.ctx, index))
        self.log(f'call   end : get_missioen_data({index})')
        return r

    def get_mission_list(self, q: ProtoInRos.Request, r: ProtoInRos.Response):
        """Return `MissionList` of all missions in dataservice (as protobuf)."""
        self.log('call begin : get_mission_list()')
        ml = MissionList(missions=[self.conv_md(md) for md in self.ds.list_missions()],
                         current_active_mission_id=self.ds.current_active_mission_id())
        r.proto = to_str(ml)
        self.log('call   end : get_mission_list()')
        return r

    def set_active_mission(self, q: GenericPrim.Request, r: GenericPrim.Response):
        """Return `success` `True` or `False`."""

        self.log(f'call begin : set_active_mission({q.data_int})')
        r.data_bool = self.ds.set_active_mission(q.data_int) is not None
        self._pub_curr_act_mi()  # update immediately
        self.log(f'call   end : set_active_mission({q.data_int})')
        return r

    def unset_active_mission(self, q: Trigger.Request, r: Trigger.Response):
        """Return `success` `True` or `False`."""

        self.log('call begin : unset_active_mission)')
        r.success, r.message = self.ds.unset_active_mission()
        self._pub_curr_act_mi()  # update immediately
        self.log('call   end : unset_active_mission)')
        return r

    def import_mission(self, q: GenericPrim.Request, r: GenericPrim.Response):
        """Return `-1` when failing, else index `[int]` of created mission."""

        self.log(f'call begin : import_mission({q.data_str})')
        r.data_int = self.ds.import_mission(q.data_str)
        self.log(f'call   end : import_mission({q.data_str})')
        return r

    def get_obj_file(self, q: GenericPrim.Request, r: GenericPrim.Response):

        self.log(f'call begin : get_obj_file({q.data_str})')
        """Return full-path-file-name from obj-file of req.mission."""
        r.data_str = self.ds.get_obj_file(q.data_int)
        self.log(f'call   end : get_obj_file({q.data_str})')
        return r
