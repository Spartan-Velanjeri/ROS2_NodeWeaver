# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

import inspect
from os import environ
from os.path import join
from pathlib import Path

from ccu_bautiro.NodeTree.NodeTree import CDrillHole, CDrillMask, CFastener, CNode
from ccu_bautiro.Types.Types import CMatrix
from ccu_dataservice.conf import Conf
from ccu_dataservice.eutil import ResourceHandler
from ccu_dataservice.loader import WorkplanLoader
from pkg_resources import resource_filename


def __create_temp_data_folder() -> str:
    test_dir = join(environ['HOME'], '__DELETE_ME__pytest__sg82fe')
    Path(test_dir).mkdir(parents=True, exist_ok=True)
    environ['CCU_DATA_DIR'] = test_dir
    return Conf().data_folder_path


def test_load_workplan_from_taskplan():
    print('### RUNNING: '+inspect.currentframe().f_code.co_name.upper())

    tmp_folder = __create_temp_data_folder()
    candidate = WorkplanLoader(ResourceHandler(tmp_folder))

    tp_xml = resource_filename('ccu_dataservice', 'mission_files_1/TaskPlan-2.1.xml')
    wp = candidate.load_workplan_from_taskplan(tp_xml)
    assert 14 == len(wp.jobs)
    assert 10 == len(wp.jobSchedule)
    assert wp.jobSchedule[0].name == 'DrillJob.1'
    assert wp.jobSchedule[1].name == 'DrillJob.2'
    assert wp.jobSchedule[2].name == 'DrillJob.3'
    assert wp.jobSchedule[3].name == 'DrillJob.4'
    assert wp.jobSchedule[8].name == 'DrillJob.9'
    assert wp.jobSchedule[9].name == 'MoveJob.1'

    mvj = wp.jobSchedule[9]
    assert mvj.id == 'vj::MoveJob.1'

    assert 1 == len(wp.nodeTree)
    node_lvl_0 = wp.nodeTree[0]
    assert 1 == len(node_lvl_0.children)
    node_lvl_1: CNode = node_lvl_0.children[0]
    assert 'P.5.0' == node_lvl_1.name
    assert 'n::0.0' == node_lvl_1.id
    node_lvl_2: CNode = node_lvl_1.children[9]
    assert 'n::0.0.9' == node_lvl_2.id
    assert 'P.5.0.9' == node_lvl_2.name
    tm: CMatrix = node_lvl_2.tm
    assert -8.660254 == tm.v0x
    assert -0.866 == tm.v1x
    assert -0.5 == tm.v2x
    assert -0.866 == tm.v2y
    f: CFastener = node_lvl_2.fastener
    assert 'n::0.0.9::f' == f.id
    assert 'GPR 1/2_37289' == f.name
    assert 0.0 == f.dx
    assert 0.0 == f.dy
    dm: CDrillMask = f.drillMasks[0]
    assert 'n::0.0.9::f::dm::0' == dm.id
    assert 'P.5.0.9.D.1' == dm.name
    dh: CDrillHole = dm.drillHoles[0]
    assert 'H.1' == dh.name
    assert 'n::0.0.9::f::dm::0::dh::0' == dh.id
    assert -0.0249 == dh.tm.v0x

    print('### SUCCESS: '+inspect.currentframe().f_code.co_name.upper())


if __name__ == '__main__':
    test_load_workplan_from_taskplan()
