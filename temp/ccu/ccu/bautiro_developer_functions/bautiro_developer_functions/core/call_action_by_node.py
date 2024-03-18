# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from typing import Any, Tuple
from threading import Event
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup as CbG
from rclpy.action.client import ClientGoalHandle
from rclpy.task import Future
from rclpy.action.server import GoalStatus

from bautiro_developer_functions.core.node_delegator import NodeDelegator


_TIMEOUT_CONNECT_TO_ACTION_SERVER = 3.0   # 3 sec
_TIMEOUT_GOAL_ACCEPTED_REQUEST = 5.0      # 5 sec
_TIMEOUT_RESULT_RETURNED_DEFAULT = 300.0  # 5 min


def call_action_fire_and_forget(ac_type: type,
                                ac_srv_name: str,
                                goal: Any,
                                *,
                                node: Node) -> str:
    nd = NodeDelegator(node)
    cgh_ac = __common_call(nd, ac_type, ac_srv_name, goal)
    if isinstance(cgh_ac, str):
        return cgh_ac  # Error case

    cgh, ac = cgh_ac
    f2: Future = cgh.get_result_async()
    f2.add_done_callback(lambda future: ac.destroy())
    # TODO  ac.destroy()  here ?
    return nd.info(f'{ac_srv_name}: Goal accepted - Fire & Forget - Not Waiting for Result')


def call_action_fire_and_forget_bool(ac_type: type,
                                     ac_srv_name: str,
                                     goal: Any,
                                     *,
                                     node: Node) -> bool:
    nd = NodeDelegator(node)
    cgh_ac = __common_call(nd, ac_type, ac_srv_name, goal)
    if isinstance(cgh_ac, str):
        return False
    cgh, ac = cgh_ac
    f2: Future = cgh.get_result_async()
    f2.add_done_callback(lambda future: ac.destroy())
    # TODO  ac.destroy()  here ?
    return True


def call_action_str(ac_type: type,
                    ac_srv_name: str,
                    goal: Any,
                    timeout: float,
                    *,
                    node: Node) -> str:
    nd = NodeDelegator(node)
    result_status = __call_action_result_status(nd, ac_type, ac_srv_name, goal, timeout)
    if isinstance(result_status, str):
        return result_status  # Error case
    result, status = result_status
    return status


def call_action_result(ac_type: type,
                       ac_srv_name: str,
                       goal: Any,
                       timeout: float,
                       *,
                       node: Node) -> Any:
    nd = NodeDelegator(node)
    result_status = __call_action_result_status(nd, ac_type, ac_srv_name, goal, timeout)
    if isinstance(result_status, str):
        return None
    result, status = result_status
    return result  # can be None


def __call_action_result_status(nd: NodeDelegator,
                                ac_type: type,
                                ac_srv_name: str,
                                goal: Any,
                                timeout: float) -> Tuple[Any, str]:
    cgh_ac = __common_call(nd, ac_type, ac_srv_name, goal)
    if isinstance(cgh_ac, str):  # Error case
        return cgh_ac

    cgh, ac = cgh_ac
    nd.info(f'{ac_srv_name}: Goal accepted - Waiting for Result')

    # Future 2 - long running stuff
    f2_done = Event()
    f2: Future = cgh.get_result_async()
    f2.add_done_callback(lambda future: f2_done.set())
    timeout = timeout or _TIMEOUT_RESULT_RETURNED_DEFAULT
    if not f2_done.wait(timeout):
        ac.destroy()
        return nd.warn(f'No goal response within {int(timeout/60)} min. from "{ac_srv_name}" ')

    result = f2.result().result
    status = f2.result().status
    ac.destroy()
    if status == GoalStatus.STATUS_SUCCEEDED:
        status_str = nd.info(f'Goal succeeded {ac_srv_name}: {repr(result)}')
    else:
        status_str = nd.warn(f'Goal failed [state: {status}] {ac_srv_name}: {repr(result)}')
    return result, status_str


def __common_call(nd: NodeDelegator,
                  ac_type: type,
                  ac_srv_name: str,
                  goal: Any) -> Tuple[ClientGoalHandle, ActionClient]:

    ac = ActionClient(nd.node, ac_type, ac_srv_name, callback_group=CbG())

    nd.debug(f'Wait for action_server "{ac_srv_name}" ...')
    if not ac.wait_for_server(timeout_sec=_TIMEOUT_CONNECT_TO_ACTION_SERVER):
        ac.destroy()
        return nd.warn(f'Cannot connect to action_server "{ac_srv_name}" ')

    nd.log(f'Send "{ac_type.__qualname__}.GOAL" to server {ac_srv_name} ...')

    # Future 1 - short running: Goal accept / reject
    f1_done = Event()
    f1 = ac.send_goal_async(goal)
    f1.add_done_callback(lambda future: f1_done.set())
    if not f1_done.wait(timeout=_TIMEOUT_GOAL_ACCEPTED_REQUEST):
        return nd.warn(f'No GOAL-ACCEPT|REJECT response from "{ac_srv_name}"')

    client_goal_handle: ClientGoalHandle = f1.result()
    if not client_goal_handle.accepted:
        ac.destroy()
        return nd.warn(f'{ac_srv_name}: Goal rejected')

    return client_goal_handle, ac
