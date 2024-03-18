# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

import inspect

from ccu_dataservice.conf import Conf
from ccu_dataservice.core import DSContext, HcuDS


def ctx():
    conf = Conf('__DELETE_ME__pytest__sg82fe')
    context = DSContext(conf)
    return context


def test_list_missions():
    print('### RUNNING: '+inspect.currentframe().f_code.co_name.upper())
    hcu = HcuDS(ctx())

    assert 1 == len(hcu.list_missions())
    assert 'TaskPlan-2.1.xml' == hcu.get_mission().name

    hcu.ctx._clean()
    print('### SUCCESS: '+inspect.currentframe().f_code.co_name.upper())


def __run_all_tests():
    test_list_missions()


if __name__ == '__main__':
    __run_all_tests()


# def do_test_get_next_drilljob(core: CoreDS):
#     print('### RUNNING: '+inspect.currentframe().f_code.co_name)
#     j: CDrillJob = core.get_next_drilljob()
#     assert 1 == len(j.pattern)


# def do_test_add_drill_event_and_update_workplan(core: CoreDS):
#     print('### RUNNING: '+inspect.currentframe().f_code.co_name)
#     j: CDrillJob = core.get_next_drilljob()
#     p = j.pattern[0]
#     drillhole0_from_pattern0_from_current_job_in_active_mission = p.drillHole[0]
#     id_test_candidate: str = drillhole0_from_pattern0_from_current_job_in_active_mission.id
#     single_drill_event = (id_test_candidate, 3)  # 3 = FINISHED from ENUM DRILL_HOLE_STATE
#     drill_events_list = [single_drill_event]

#     core.add_drill_event_and_update_workplan(drill_events_list)

#     mission_content = core.ctx.mim.get_mission_content()

#     root = mission_content.workPlan.eContainer()

#     for dh in root.drillHole:
#         if dh.id == id_test_candidate:
#             if dh.state.value == 3:
#                 found = True
#     assert found
