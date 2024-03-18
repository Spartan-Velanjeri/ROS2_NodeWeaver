# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from inspect import currentframe as cf
from time import time
from typing import List

from ccu_bautiro.Jobs.Jobs import CDrillJob
from ccu_bautiro.NodeTree.NodeTree import CDrillHole, CDrillMask
from ccu_dataservice.conf import Conf
from ccu_dataservice.core import CoreDS, DSContext
from ccu_dataservice.manager import DrillHoleResult
from pyecore.resources.xmi import XMIOptions


def __begin(cf):
    print('### RUNNING: ' + cf.f_code.co_name.upper())


def __end(cf):
    print('### SUCCESS: ' + cf.f_code.co_name.upper())


# @pytest.fixture


def ctx():
    Conf('__DELETE_ME__pytest__sg82fe')._clean()
    context = DSContext(Conf('__DELETE_ME__pytest__sg82fe'))
    return context


XMI_SAVE_OPTIONS = {XMIOptions.SERIALIZE_DEFAULT_VALUES: True,
                    XMIOptions.OPTION_USE_XMI_TYPE: True}


def test_get_next_drill_jobs_size():

    __begin(cf())
    core = CoreDS(ctx())
    print(0)
    assert 1 == len(core.get_next_drill_jobs())
    assert 10 == len(core.get_next_drill_jobs(300))
    assert 10 == len(core.get_next_drill_jobs(-88))
    assert 10 == len(core.get_next_drill_jobs(0))


def test_get_next_drill_jobs_time_measure():

    __begin(cf())
    core = CoreDS(ctx())
    dj: CDrillJob
    print('======================================================================')
    dj_ids = ['dj::DrillJob.1',
              'dj::DrillJob.2',
              'dj::DrillJob.3',
              'dj::DrillJob.4',
              'dj::DrillJob.5',
              'dj::DrillJob.6',
              'dj::DrillJob.7',
              'dj::DrillJob.8',
              'dj::DrillJob.9',
              ]
    for expected_id in dj_ids:
        a = time()
        dj = core.get_next_drill_jobs()[0]
        assert dj.id == expected_id
        print(dj.id)
        b = time()
        print(f'get_next_drill_jobs  duration: {(b-a):.3f} s.')
        drill_event = create_drillevent_with_drillhole_state_drilled(dj)
        c = time()
        print(f'create_drillevent..  duration: {(c-b):.3f} s.    [{len(drill_event)} elements]')
        core.add_mission_event(drill_event=drill_event)
        d = time()
        print(f'add_mission_event    duration: {(d-c):.3f} s.')
        print('--------------------------------------------------------------------')

    assert 0 == len(core.get_next_drill_jobs(300))
    __end(cf())
    core.destroy()
    core._clean()


DRILLED = 3


def create_drillevent_with_drillhole_state_drilled(dj: CDrillJob):
    result: List[DrillHoleResult] = []

    dh: CDrillHole
    dm: CDrillMask
    for dm in dj.drillMasks:
        for dh in dm.drillHoles:
            dhr = DrillHoleResult(id=dh.id, state_val=DRILLED, pose_start=None, pose_finish=None)
            result.append(dhr)
    return result


def __run_all_tests():
    # test_get_next_drill_jobs()
    test_get_next_drill_jobs_time_measure()


if __name__ == '__main__':
    __run_all_tests()
