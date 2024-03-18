# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.


from os import scandir
from os.path import basename, join, isfile
from pathlib import Path
from typing import Dict, List, Tuple, Any
from shutil import copy2, move
from datetime import datetime
from hashlib import md5
from enum import Enum

from xml.etree import ElementTree
from xml.etree.ElementTree import Element


from pyecore.ecore import EEnumLiteral
from pyecore.resources.xmi import XMIResource

from ccu_bautiro.ccu_bautiro import CMission, CProcessData, CWorkPlan
from ccu_bautiro.Jobs import CJob, CDrillJob, CMeasureJob
from ccu_bautiro.MetaData import CMetaData, CMissionMD, CTaskPlanMD, CWorkPlanMD, CRoomPlanMD
from ccu_bautiro.NodeTree import CDrillHole
from ccu_bautiro.ProcessData import CDrillHolePD, CNavigationPD
from ccu_bautiro.Types import CPose, DRILL_HOLE_STATE, IMPORT_SOURCE, TP_XSD_VERSION

from ccu_dataservice.conf import Conf
from ccu_dataservice.eutil import XMI_SAVE_OPTIONS, ResourceHandler, is_same_classname
from ccu_dataservice.loader import WorkplanLoader
from ccu_dataservice.validator import TPValidator

Q = Conf.FQN_SEPERATOR


class BaseManager():
    """creates folder-structure:..../data/<managed_folder>/<index>/<data-file.extension>."""

    def __init__(self, md_type: type, rh: ResourceHandler):
        self.md_class = rh.e_class(md_type)
        self.rh = rh

        # names
        n: str = self.md_class.name.lower()
        # roomplan | workplan | taskplan | mission
        self.name = n[1:n.index('md')]
        # rp, wp, tp, mi
        self.prefix = (self.name[0] + 'p') if "plan" in self.name else self.name[0:2]
        self.managed_folder_name = self.name + 's'
        self.managed_folder_rel_path = self.managed_folder_name
        self.managed_folder_abs_path = join(rh.data_folder_path, self.managed_folder_rel_path)

        self.data_filename = self.name + '.xml.ccu_bautiro'  # valid for CMission and CWorkPlan
        self.md_filename = self.name + '.metadata.xml.ccu_bautiro'

        # business logic
        self.dict = self.update_dict()

    @property
    def temp_folder(self):
        return join(self.managed_folder_abs_path, 'tmp')

    def is_valid_index(self, index) -> bool:
        return index in self.dict.keys()

    def get_data_file_rel_path(self, index: int) -> str:
        return join(self.managed_folder_name, str(index), self.data_filename)

    def get_xmi_resource(self, index: int) -> XMIResource:
        return self.rh.get_resource_rel(self.get_data_file_rel_path(index))

    def get_md_file_rel_path(self, index: int) -> str:
        return join(self.managed_folder_name, str(index), self.md_filename)

    def get_data_file_abs_path(self, index: int) -> str:
        return join(self.managed_folder_abs_path, str(index), self.data_filename)

    # CRoot -or- CMission -or- CWorkPlan ... depends on ...
    def get_data_file_content(self, index: int) -> Any:
        if self.is_valid_index(index):
            res: XMIResource = self.get_xmi_resource(index)
            # res.load()
            return res.contents[0]

    def get_metadata(self, index: int = None) -> CMetaData:
        if self.is_valid_index(index):
            return self.dict[index]

    def get_md_file(self, index) -> XMIResource:
        return self.rh.get_resource_rel(self.get_md_file_rel_path(index))

    def update_dict(self) -> Dict[int, CMetaData]:
        """Build up dict: { 0: CMetaData('.../data/taskplans/0/taskplan.metadata'), ... }."""
        self.dict: Dict[int, CMetaData] = {}
        if self.managed_folder_abs_path:
            Path(self.managed_folder_abs_path).mkdir(parents=True, exist_ok=True)
            for sub in scandir(self.managed_folder_abs_path):
                if sub.name.isdigit() and sub.is_dir():  # consider '1', '3', '77'
                    index = int(sub.name)
                    try:
                        self.dict[index]: CMetaData = self.get_md_file(index).contents[0]
                    except Exception:
                        print(f"Dict-MetaData-issue at {self.name}-manager")
        return self.dict

    def import_data_file(self, data_file_full_path, *args) -> CMetaData:
        """
          data_file_full_path :   /tmp/tmp_dir/sample_file.extension
          renamed-target-file :   $HOME <data> / <type> / <index> / <data-file-name>
                 tp-meta-data :   $HOME <data> / <type> / <index> / metadata.xml.ccu_bautiro
        """
        index = self.next_free_index()
        self.move_source_to_target_and_rename(data_file_full_path, index)
        md = self.create_md(data_file_full_path, index)
        md = self.customize_md(md, *args)
        md.eResource.save(options=XMI_SAVE_OPTIONS)
        self.update_dict()
        return md

    def next_free_index(self) -> int:
        """returns: `(1 + current-maximum-index)`."""
        if self.dict:
            integer_list = self.dict.keys()
            r = 1 + max(integer_list)
            return r
        return 1

    def customize_md(self, md: CMetaData, *args) -> CMetaData:
        """Shall be overwritten by sub-classes."""
        return md

    def create_md(self, data_file_full_path, index: int, import_source=None) -> CMetaData:
        """creates and pre-filles any CMetaData instance."""
        md: CMetaData = self.md_class()
        md.index = index
        md.originFilePath = data_file_full_path
        md.originFileName = basename(data_file_full_path)
        md.id = f'md{Q}{self.prefix}.{index}'
        md.relPath = join(self.managed_folder_name, str(index), self.data_filename)
        md.dateTimeCreation = datetime.now().isoformat()
        md.originImportSource = import_source or self.rh.e_enum(IMPORT_SOURCE).from_string('HCU')
        self.rh.create_resource_rel(self.get_md_file_rel_path(index)).append(md)
        return md

    def copy_source_to_intermediate(self, source):
        Path(self.temp_folder).mkdir(parents=True, exist_ok=True)
        intermediate = join(self.temp_folder, self.data_filename)
        copy2(source, intermediate)
        return intermediate

    def copy_intermediate_file_to_target_dir(self, intermediate, target_dir):
        Path(target_dir).mkdir(parents=True, exist_ok=True)
        target = join(target_dir, self.data_filename)
        copy2(intermediate, target)
        return target

    def move_source_to_target_and_rename(self, source_file, index: int):
        target_dir = join(self.managed_folder_abs_path, str(index))
        Path(target_dir).mkdir(parents=True, exist_ok=True)
        target_file = join(target_dir, self.data_filename)
        move(source_file, target_file)
        return target_file


class RoomplanManager(BaseManager):

    EXTENSION_XML = 'xml'

    def __init__(self, rh: ResourceHandler):
        super().__init__(CRoomPlanMD, rh)
        # override parent attrib -> 'roomplan.xml'
        self.data_filename = self.name + '.obj'

    def get_metadata(self, index: int) -> CRoomPlanMD:
        return super().get_metadata(index)

    def import_roomplan_by_move(self, rp_full_file_path, *args) -> CRoomPlanMD:
        """ rp_full_file_path :  /tmp/transfer_dir/sample_roomplan.obj
                       TARGET :  /home/bautiro/ data/ roomplans/ 7/ roomplan.obj
                 wp-meta-data :  /home/bautiro/ data/ roomplans/ 7/ metadata.xml.ccu_bautiro    """
        return super().import_data_file(rp_full_file_path, *args)

    def customize_md(self, md: CRoomPlanMD, *args) -> CRoomPlanMD:

        def calculate_md5(file_abs_path):
            hash_md5 = md5()
            try:
                with open(file_abs_path, "rb") as f:
                    for chunk in iter(lambda: f.read(4096), b""):
                        hash_md5.update(chunk)
                return hash_md5.hexdigest()
            except IOError as e:
                print(f'IOError on opening file {file_abs_path}: {e}')
                return '0' * 32

        md.calculated_MD5_32bitHex = calculate_md5(self.get_data_file_abs_path(md.index))
        return md


class TaskplanManager(BaseManager):

    EXTENSION_XML = 'xml'

    def __init__(self, rh: ResourceHandler, validator: TPValidator = None):
        super().__init__(CTaskPlanMD, rh)
        # override parent attrib -> 'taskplan.xml'
        self.data_filename = self.name + '.xml'
        self.validator = validator

    def get_metadata(self, index: int) -> CTaskPlanMD:
        return super().get_metadata(index)

    def list_taskplan_on_ccu(self) -> List[str]:
        """Return a list of the 'name'(`.originFileName`) of all `<TaskPlan.XML>` available."""
        result: List[str] = []
        for value in self.dict.values():
            md: CTaskPlanMD = value
            result.append(md.originFileName or f'unset originFileName @ taskplan id: {md.id}')
        return result

    def import_taskplan_by_move(self, tp_path, *args) -> CTaskPlanMD:
        """
          tp_full_file_path :   /tmp_dir/sample_taskplan.xml
        renamed-target-file :   $HOME / <data> /taskplans/ <index> / taskplan.xml
               tp-meta-data :   $HOME / <data> /taskplans/ <index> / metadata.xml.ccu_bautiro
        """
        return super().import_data_file(tp_path, *args)

    def customize_md(self, md: CTaskPlanMD, *args) -> CTaskPlanMD:
        md.name = md.originFileName  # What "name" ?
        if args:
            md.roomPlanIndex = args[0]

        tp20: Element = ElementTree.parse(self.get_data_file_abs_path(md.index)).getroot()
        if s := tp20.find('GlobalSettings'):
            md.globalSettings_RoomPlan_Filename = s.find('RoomPlan/Filename') or ''
            md.globalSettings_RoomPlan_MD5_32bitHex = s.find('RoomPlan/MD5') or ''
            md.globalSettings_Schedule_desiredStartTime = s.find('Schedule/desiredStartTime') or ''
        return md

    def copy_source_to_intermediate(self, source):
        Path(self.temp_folder).mkdir(parents=True, exist_ok=True)
        intermediate = join(self.temp_folder, self.data_filename)
        copy2(source, intermediate)
        return intermediate

    def copy_intermediate_file_to_target_dir(self, intermediate, target_dir):
        Path(target_dir).mkdir(parents=True, exist_ok=True)
        target = join(target_dir, self.data_filename)
        copy2(intermediate, target)
        return target

    def create_tp_md(self, origin_path: str, index: int):
        tp_md: CTaskPlanMD = super().create_md(origin_path, index)
        # TODO what is the 'name' of Task, Work, Mission
        tp_md.name = basename(origin_path)
        tp_md.originImportSource = self.rh.e_enum(IMPORT_SOURCE).from_string('HCU')
        tp_md.taskPlanXsdVersion = self.rh.e_enum(TP_XSD_VERSION).from_string('TaskPlan_2_0')
        return tp_md


class DrillHoleResult():
    def __init__(self, id: str, state_val: int, pose_start: Tuple, pose_finish: Tuple) -> None:
        self.id: str = id
        self.state_val: int = state_val
        self.pose_start: Tuple = pose_start
        self.pose_finish: Tuple = pose_finish

    def update_dhr_pd(self, dh: CDrillHole, pd: CDrillHolePD, c_pose_type: type) -> CDrillHolePD:

        pd.drillHole = dh

        if self.pose_start:
            s: CPose = c_pose_type()
            s.vx, s.vy, s.vz = self.pose_start[0]
            s.qx, s.qy, s.qz, s.qw = self.pose_start[1]
            pd.absPosePKSStart = s

        if self.pose_finish:
            f: CPose = c_pose_type()
            f.vx, f.vy, f.vz = self.pose_finish[0]
            f.qx, f.qy, f.qz, f.qw = self.pose_finish[1]
            pd.absPosePKSFinish = f

        return pd


class NavResult():
    def __init__(self, cluster_id: str, nav_goal_reached: bool) -> None:
        self.cluster_id: str = cluster_id
        self.nav_goal_reached: bool = nav_goal_reached

    def update_nr_pd(self, cj: CJob, pd: CNavigationPD):
        pd.clusterJob = cj
        pd.nav_goal_reached = self.nav_goal_reached
        return pd


class WorkplanManager(BaseManager):
    """Internal CCU representation of 'Taskplan'  -> (potentially) editable."""

    EXTENSION_PD = "processing_data.xml.ccu_bautiro"

    def __init__(self, rh: ResourceHandler, loader: WorkplanLoader):
        super().__init__(CWorkPlanMD, rh)
        self.loader = loader

    def import_taskplan_to_workplan(self,
                                    tp_md: CTaskPlanMD,
                                    rp_index: int) -> Tuple[CWorkPlan, CWorkPlanMD]:

        tp_abs_path = self.rh.get_abs_path(tp_md.relPath)
        wp = self.loader.load_workplan_from_taskplan(tp_abs_path)
        wp.index = self.next_free_index()
        wp.id = f'wp{Q}{wp.index}'
        self.rh.create_resource_rel(self.get_data_file_rel_path(wp.index)).append(wp)
        wp.eResource.save(options=XMI_SAVE_OPTIONS)
        # TODO prefix all  <ID>  element --> '/wp:23/'

        wp_md = self.create_md(tp_md.name, wp.index)
        self.customize_md(wp_md, rp_index, tp_md.index)
        wp_md.eResource.save(options=XMI_SAVE_OPTIONS)
        self.dict[wp.index] = wp_md
        return wp, wp_md

    def customize_md(self, wp_md: CWorkPlanMD, *args) -> CWorkPlanMD:
        if args:
            rp_index = args[0]
            wp_md.roomPlanIndex = rp_index
            wp_md.taskPlanIndex = args[1] if (len(args) > 1 and args[1] > 0) else -1
        return wp_md

    def get_metadata(self, index: int) -> CWorkPlanMD:
        return super().get_metadata(index)

    def get_data_file_content(self, wp_index: int) -> CWorkPlan:
        return super().get_data_file_content(wp_index)

    def add_mission_event(self, mi_index: int, wp: CWorkPlan,
                          dh_results: List[DrillHoleResult] = None,
                          n_result: NavResult = None) -> None:

        if (dh_results is None) and (n_result is None):
            return  # TODO log something about 'empty' event

        pd: CProcessData = self.create_pd(wp.index, mi_index)
        if n_result:
            nav_pd = self.create_nav_pd(n_result, wp.jobs)
            nav_pd.id = f"{pd.id}{Q}n"
            pd.navigationPD.append(nav_pd)
        if dh_results:
            for dhr in dh_results:
                dh_pd = self.create_dh_pd(dhr, wp.drillHoles)
                dh_pd.id = f"{pd.id}{Q}dh({dh_pd.drillHole.id})"
                pd.drillHolePD.append(dh_pd)
        pd.eResource.save(options=XMI_SAVE_OPTIONS)
        wp.eResource.save(options=XMI_SAVE_OPTIONS)

    def create_dh_pd(self, dhr: DrillHoleResult, drillholes: List[CDrillHole]):
        dh: CDrillHole = next((dh for dh in drillholes if dhr.id == dh.id), None)
        if not dh:
            raise Exception(f"DrillHole id: '{dhr.id}' does not exist")
        dh.state = dh.state.eEnum.getEEnumLiteral(value=dhr.state_val)
        return dhr.update_dhr_pd(dh, self.rh.e_class(CDrillHolePD)(), self.rh.e_class(CPose))

    def create_nav_pd(self, nr: NavResult, cluster_jobs: List[CJob]):
        j: CDrillJob = next(
            (j for j in cluster_jobs if nr.cluster_id == j.id), None)
        if not j:
            raise Exception(f"JobCluster.id: '{nr.cluster_id}' does not exist")
        return nr.update_nr_pd(j, self.rh.e_class(CNavigationPD)())

    def next_free_pd_event_index(self, wp_index) -> int:
        pd_events = self.get_processing_data_event_indexes(wp_index)
        if pd_events:
            return 1 + max(pd_events)
        return 1

    def get_processing_data_event_indexes(self, wp_index: int) -> List[int]:
        """Return [1,2,3,..,n]  for corresponding `<n>.<processingdata_ext>`."""
        indexes: List[str] = []
        if self.is_valid_index(wp_index):
            workplan_dir_i = join(self.managed_folder_abs_path, str(wp_index))
            for file in scandir(workplan_dir_i):
                if file.name.endswith(self.EXTENSION_PD):
                    s = file.name.replace(f'.{self.EXTENSION_PD}', '')
                    indexes.append(int(s))
        return indexes

    def create_pd(self, wp_index: int, mi_index: int) -> CProcessData:
        pd: CProcessData = self.rh.e_class(CProcessData)()
        pd.index = self.next_free_pd_event_index(wp_index)
        pd.id: str = f"pd.{pd.index}(wp.{wp_index}{Q}mi.{mi_index})"
        pd.missionIndex = mi_index
        pd.workPlanIndex = wp_index
        pd.dateTimeCreation = datetime.now().isoformat()
        self.rh.create_resource_rel(self.get_pd_file_rel_path(wp_index, pd.index)).append(pd)
        return pd

    def get_pd_file_rel_path(self, wp_index: int, pd_index: int) -> str:
        return join(self.managed_folder_rel_path, f'{wp_index}', f'{pd_index}.{self.EXTENSION_PD}')


class MissionManager(BaseManager):

    def __init__(self, rh: ResourceHandler):
        super().__init__(CMissionMD, rh)
        self.active_mission_file = join(
            self.managed_folder_abs_path, 'active_mission.txt')
        self.active_mission_index = self.read_active_mission_index_from_file()

    def is_mission_active(self) -> bool:
        return self.is_valid_index(self.active_mission_index)

    def get_metadata(self, index: int = None) -> CMissionMD:
        return super().get_metadata(index or self.active_mission_index)

    def get_data_file_content(self, index: int = None) -> CMission:
        return super().get_data_file_content(index or self.active_mission_index)

    def set_active_mission(self, index: int, force: bool = False) -> CMissionMD:
        """Return `None` when given index is invalid or corresponding `CMissionMD`."""
        if self.is_valid_index(index) or force:
            self.active_mission_index = index
            self.write_active_mission_index_to_file(index)
            return self.get_metadata(index)

    def unset_active_mission(self) -> Tuple[bool, str]:
        old = self.active_mission_index
        self.active_mission_index = -1
        self.write_active_mission_index_to_file(-1)
        if old < 1:
            return False, 'Warn: cannot deactivate none-active mission'
        return True, f'OKAY. Succesfully deactivatet Mission: {old}'

    def create_mission(self, workplan: CWorkPlan, tp_name: str, wp_index) -> CMissionMD:
        index = self.next_free_index()
        mi: CMission = self.rh.e_class(CMission)()
        mi.index = index
        mi.id = f"{self.prefix}.{mi.index}"
        self.rh.create_resource_rel(self.get_data_file_rel_path(index)).append(mi)
        for j in workplan.jobSchedule:
            mi.jobs.append(j)

        md: CMissionMD = super().create_md(self.get_data_file_abs_path(index), mi.index)
        md.name = tp_name
        md.workPlanIndex = wp_index
        md.eResource.save(options=XMI_SAVE_OPTIONS)
        self.dict[mi.index] = md
        mi.eResource.save(options=XMI_SAVE_OPTIONS)
        return md

    def read_active_mission_index_from_file(self) -> int:
        if isfile(self.active_mission_file):
            with open(self.active_mission_file, 'r') as f:
                number = int(f.read())
                f.close()
                return number
        else:
            return self.write_active_mission_index_to_file(-1)

    def write_active_mission_index_to_file(self, index: int) -> int:
        with open(self.active_mission_file, 'w') as f:
            f.write(str(index))
            f.close
        return index

    def get_all_cluster_jobs(self) -> List[CJob]:
        return self.get_data_file_content().jobs

    def get_next_cluster_jobs(self, limit: int = 1) -> List[CJob]:
        """Next Jobs as nav-targets for RPM."""
        def is_job_todo(job: CJob) -> bool:
            """Return `True` for CDrillJob in case 1 (or more) hole is `UNDRILLED` else 'False`."""
            if is_same_classname(job, CDrillJob):
                return any(is_undrilled(dh) for dm in job.drillMasks for dh in dm.drillHoles)
            elif is_same_classname(job, CMeasureJob):
                return False
            return False

        def is_undrilled(dh: CDrillHole):
            curr: EEnumLiteral = dh.state
            comp: EEnumLiteral = DRILL_HOLE_STATE.getEEnumLiteral('UNDRILLED')
            return curr.value == comp.value

        if mission := self.get_data_file_content():
            return list(filter(is_job_todo, mission.jobs))[0:limit]
        return []
