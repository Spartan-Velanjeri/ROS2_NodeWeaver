# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from typing import List
from threading import Event
from bautiro_developer_functions.core.node_delegator import NodeDelegator

from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup as CbG
from rclpy.task import Future
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from action_msgs.msg import GoalStatus

from bautiro_ros_interfaces.action import StartFpmSkill
from bautiro_ros_interfaces.msg import Marker, DrillMask


_WAIT_FOR_SERVER_TIMEOUT = 5.0  # seconds
_GOAL_ACCEPTED_TIMEOUT = 10.0   # seconds


class FpmSkillCall(NodeDelegator):

    def __init__(self, node: Node) -> None:
        super().__init__(node)

        self.ac = ActionClient(node, StartFpmSkill, '/fpm_skill', callback_group=CbG())

        # business logic:
        self.busy = False
        self.result_success_state = False
        self.result_received_event = Event()

    def destroy(self):
        self.ac.destroy()

    def call_skill(self, skill_xml: str,
                   drill_masks: List[DrillMask] = [],
                   lift_setpoint_position: float = None,
                   platform_ceiling_distance: float = None,
                   handling_unit_configured_pose: int = None,
                   markers: List[Marker] = []) -> bool:
        goal = StartFpmSkill.Goal(skill_name=skill_xml,
                                  drill_masks=drill_masks)
        if self.busy:
            self.warn("action client busy!")
            return False
        self.busy = True  # this blocks subsequent call (double-click)
        if not self.ac.wait_for_server(timeout_sec=_WAIT_FOR_SERVER_TIMEOUT):
            self.warn("no reply from Action_Server: /fpm_skill")
            self.busy = False
            return False

        f1_done = Event()
        f1 = self.ac.send_goal_async(goal, self._progress_cb)
        f1.add_done_callback(lambda future: f1_done.set())
        if not f1_done.wait(timeout=_GOAL_ACCEPTED_TIMEOUT):
            self.busy = False
            return False

        goal_request: ClientGoalHandle = f1.result()
        if not goal_request.accepted:
            self.busy = False
            return False

        self.info('Goal accepted')
        f2: Future = goal_request.get_result_async()
        f2.add_done_callback(self._result_callback)
        self.result_received_event.clear()
        return True

    def _result_callback(self, future: Future):
        self.result_success_state = (GoalStatus.STATUS_SUCCEEDED == future.result().status)
        self.result_received_event.set()  # 👉 yield progress in gRPC-Manual-SetState
        self.info(f'Result received, success: {self.result_success_state}')
        self.busy = False

    def _progress_cb(self, *args):
        self.info("Action Server '/fpm_skill' progress received (no forward to gRPC)")


class FpmSkillCallWrapper(NodeDelegator):

    def __init__(self, fsc: FpmSkillCall):
        super().__init__(fsc.node)
        self.fsc = fsc

    @property
    def success(self) -> bool:
        return self.fsc.result_success_state

    def finished(self) -> bool:
        return self.fsc.result_received_event.is_set()

    def call_skill(self, skill_xml: str, **kwargs) -> bool:
        return self.fsc.call_skill(skill_xml=skill_xml, **kwargs)

    def wait_for_result(self, timeout: float) -> bool:
        return self.fsc.result_received_event.wait(timeout=timeout)
