# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from shutil import copytree
from os import scandir
from os.path import join, basename, abspath
from typing import List, Tuple, Optional


from ccu_bautiro.ccu_bautiro import CMission
from ccu_bautiro.Jobs import CJob, CDrillJob
from ccu_bautiro.MetaData import CMissionMD, CTaskPlanMD
from ccu_dataservice.conf import Conf
from ccu_dataservice.eutil import ResourceHandler
from ccu_dataservice.loader import WorkplanLoader
from ccu_dataservice.manager import (DrillHoleResult, MissionManager, NavResult,
                                     RoomplanManager, TaskplanManager, WorkplanManager)
from ccu_dataservice.validator import TPValidator
from ccu_util.util import DummyLogger

###################################################################################################
#                                                                                                 #
#  DS Context Object                                                                              #
#                                                                                                 #
###################################################################################################


class DSContext():

    def __init__(self, conf: Conf = None, logger=None) -> None:
        self.logger = logger or DummyLogger()
        self.conf = conf or Conf()
        self.rh = ResourceHandler(self.conf.data_folder_path)
        self.validator = TPValidator()
        self.loader = WorkplanLoader(self.rh)
        self.rpm = RoomplanManager(self.rh)
        self.tpm = TaskplanManager(self.rh, self.validator)
        self.wpm = WorkplanManager(self.rh, self.loader)
        self.mim = MissionManager(self.rh)

    def _clean(self):
        self.conf._clean()
        return self


###################################################################################################
#                                                                                                 #
#  DS Base Class                                                                                  #
#                                                                                                 #
###################################################################################################


class DataService():

    def __init__(self, ctx: DSContext, missions_dir: str = None) -> None:
        self.ctx = ctx
        self._create_dummy_data_if_no_data_there(missions_dir)
        # self.ctx.mim.unset_active_mission()

    def _clean(self):
        self.ctx._clean()
        return self.ctx

        ###########################################################################################
        #                                                                                         #
        #  Import Mission (TaskPlan + RoomPlan)                                                   #
        #                                                                                         #
        ###########################################################################################

    def import_mission_files_to_data_dir_by_move(self, tmp_dir: str) -> int:
        """Return -1 for invalid files, else index of newly created mission."""
        tp, rp = self._get_valid_tp_rp(tmp_dir)

        if not tp:
            return -1
        rp_index = self.ctx.rpm.import_roomplan_by_move(rp).index if rp else -1
        tp_md = self.ctx.tpm.import_taskplan_by_move(tp, rp_index)
        wp, wp_md = self.ctx.wpm.import_taskplan_to_workplan(tp_md, rp_index)
        mi_md = self.ctx.mim.create_mission(wp, tp_md.name, wp_md.index)
        return mi_md.index

    def _get_valid_tp_rp(self, tmp_dir: str) -> Tuple[str, str]:
        """Return abs-path if taskplan is (XSD) valid. OPTIONAL roomplan."""

        with scandir(tmp_dir) as files:
            if tp := self._get_tp(tmp_dir):
                for f in files:
                    if f.is_file() and f.name.lower().endswith('.obj'):
                        return (tp, join(tmp_dir, f.name))
                return (tp, None)
        return (None, None)

    def _get_tp(self, tmp_dir: str) -> Optional[str]:
        with scandir(tmp_dir) as files:
            for f in files:
                if f.name.lower().endswith('.xml') and f.is_file():
                    _r = '\033[91m'  # red
                    _y = '\033[93m'  # yellow
                    _g = '\033[92m'  # green
                    _n = '\033[0m'   # no color
                    if self.ctx.validator.validate(abspath(f.path)):
                        self.ctx.logger.info(f'{_g}SUCCESS: load {f.path}{_n}')
                        return join(tmp_dir, f.name)
                    else:
                        self.ctx.logger.error(f'{_r}FAILED: load {f.path}. invalid TaskPlan{_n}')
                        self.ctx.logger.warn(f'{_y}{self.ctx.validator.error_text}{_n}')
                        return

    def _create_dummy_data_if_no_data_there(self, mission_dirs: str) -> None:
        if self.ctx.wpm.dict.keys():
            return
        if mission_dirs:
            for mission_dir in sorted(scandir(mission_dirs), key=lambda dir: dir.name):
                if mission_dir.is_dir():
                    dst = '/tmp/' + basename(mission_dir)
                    copytree(mission_dir, dst, dirs_exist_ok=True)
                    self.import_mission_files_to_data_dir_by_move(dst)


###################################################################################################
#                                                                                                 #
#  Core Sub-class                                                                                 #
#                                                                                                 #
###################################################################################################

def _ensure_integer_within_range(number_of_next):
    if number_of_next is None:
        return 1
    if number_of_next < 1:
        return Conf.NUMBER_OF_NEXT_DEFAULT
    if number_of_next > Conf.NUMBER_OF_NEXT_MAX:
        return Conf.NUMBER_OF_NEXT_MAX
    return number_of_next


class CoreDS(DataService):

    def __init__(self, ctx: DSContext, missions_dir: str = None) -> None:
        super().__init__(ctx, missions_dir)

    def all_cluster_jobs(self) -> List[CJob]:
        return self.ctx.mim.get_all_cluster_jobs()

    def get_next_drill_jobs(self, number_of_next: int = None) -> List[CJob]:
        n = _ensure_integer_within_range(number_of_next)
        return self.ctx.mim.get_next_cluster_jobs(n)

    def get_next_drilljob(self) -> CDrillJob:
        if jobs := self.ctx.mim.get_next_cluster_jobs(1):
            return jobs[0]

    def add_mission_event(self, drill_event: List[DrillHoleResult] = None,
                          nav_result: NavResult = None) -> None:
        if mi_md := self.ctx.mim.get_metadata():
            wp = self.ctx.wpm.get_data_file_content(mi_md.workPlanIndex)
            self.ctx.wpm.add_mission_event(
                mi_md.index, wp, drill_event, nav_result)


###################################################################################################
#                                                                                                 #
#  HALC Sub-class                                                                                 #
#                                                                                                 #
###################################################################################################


class HcuDS(DataService):

    def unset_active_mission(self) -> Tuple[bool, str]:
        return self.ctx.mim.unset_active_mission()

    def get_md5_tp(self, mission_index: int = None) -> Optional[str]:
        mi_md = self.ctx.mim.get_metadata(mission_index)
        if not mi_md:
            return None
        wp_md = self.ctx.wpm.get_metadata(mi_md.workPlanIndex)
        if tp_md := self.ctx.tpm.get_metadata(wp_md.taskPlanIndex):
            return tp_md.globalSettings_RoomPlan_MD5_32bitHex

    def get_md5_calc(self, mission_index: int = None) -> Optional[str]:
        if mi_md := self.ctx.mim.get_metadata(mission_index):
            wp_md = self.ctx.wpm.get_metadata(mi_md.workPlanIndex)
            tp_md = self.ctx.tpm.get_metadata(wp_md.taskPlanIndex)
            if rp_md := self.ctx.rpm.get_metadata(tp_md.roomPlanIndex):
                return rp_md.calculated_MD5_32bitHex

    def get_mission(self, mission_index: int = None) -> CMissionMD:
        return self.ctx.mim.get_metadata(mission_index)

    def current_active_mission_id(self) -> int:
        return self.ctx.mim.active_mission_index

    def set_active_mission(self, mission_index: int) -> CMissionMD:
        return self.ctx.mim.set_active_mission(mission_index)

    def import_mission(self, tmp_dir: str) -> int:
        return self.import_mission_files_to_data_dir_by_move(tmp_dir)

    def list_taskplan_on_ccu(self) -> List[CTaskPlanMD]:
        return self.ctx.tpm.dict.values()

    def list_missions(self) -> List[CMissionMD]:
        return self.ctx.mim.dict.values()

    def get_mission_content(self, index: int = None) -> CMission:
        return self.ctx.mim.get_data_file_content(index)

    def get_obj_file(self, mission_index: int) -> str:
        if mi_md := self.ctx.mim.get_metadata(mission_index):
            wp_md = self.ctx.wpm.get_metadata(mi_md.workPlanIndex)
            if rp_md := self.ctx.rpm.get_metadata(wp_md.roomPlanIndex):
                roomplan_abs_path: str = self.ctx.rh.get_abs_path(
                    rp_md.relPath)
                # 'hugo.obj:/.../4.obj'
                return f'{rp_md.originFileName}:{roomplan_abs_path}'
        return 'out_of_index.obj:/mission_does_not_exist.obj'  # 'hugo.obj:/.../4.obj'
