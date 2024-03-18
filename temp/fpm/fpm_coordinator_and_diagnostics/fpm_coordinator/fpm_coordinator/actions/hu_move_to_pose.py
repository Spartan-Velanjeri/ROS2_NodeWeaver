import time
from pathlib import Path
from typing import Any

import rclpy
from bautiro_ros_interfaces.action import FpmHuMoveToPose, StartFpmSkill
from bautiro_ros_interfaces.msg import ResponseCode
from rclpy.action import ActionClient, ActionServer
from rclpy.action.server import ActionServer, GoalStatus, ServerGoalHandle
from rclpy.node import Node

SKILL_NAME = Path(__file__).stem


class HuMoveToPose(ActionServer):
    def __init__(self, node: Node, action_type: Any, action_name: Any) -> None:
        super().__init__(
            node=node,
            action_type=action_type,
            action_name=action_name,
            execute_callback=self.action_callback,
        )

        self._coordinator_node = node
        self._logger = node.get_logger()
        self._skill_client: ActionClient = node.bt_client

        self._logger.info(f"Initialization of the skill {SKILL_NAME} done.")

    def action_callback(
        self, goal_handle: ServerGoalHandle
    ) -> FpmHuMoveToPose.Result:
        """Action server callback that implements functionality of the server.

        Args:
            goal_handle (ServerGoalHandle): Handle to the goal data structure.

        Returns:
            FpmHuMoveToPose.Result: Result of the action.
        """
        self._logger.info(
            f'Skill "{SKILL_NAME}" called with payload "{goal_handle.request}".'
        )
        # call benaviout tree skill server
        msg = StartFpmSkill.Goal()

        msg.skill_name = "fpm_handlig_unit_move_to_pose.xml"

        if goal_handle.request.pose_number not in range(0, 6):
            goal_handle.abort()
            self._logger.error("Unsuported hu pose. Valid range 0-5.")
            return FpmHuMoveToPose.Result(response_code=0)

        msg.handling_unit_configured_pose = goal_handle.request.pose_number

        # check is the skill server ready
        server_ready = self._skill_client.wait_for_server(timeout_sec=1)
        if not server_ready:
            goal_handle.abort()
            self._logger.error("Skill server time-out. Aborting action.")
            return FpmHuMoveToPose.Result(response_code=0)

        future = self._skill_client.send_goal_async(msg)

        while not future.done():
            # wait to accept or decline the request
            time.sleep(0.2)

        skill_result = future.result()

        while skill_result.status in [
            GoalStatus.STATUS_EXECUTING,
            GoalStatus.STATUS_CANCELING,
        ]:
            self._logger.info(f"{SKILL_NAME}: skill is executing.")
            time.sleep(2)

        if skill_result.status == GoalStatus.STATUS_ABORTED:
            goal_handle.abort()
            print(future.result())
            self._logger.error("Skill server aborted the skill.")
            return FpmHuMoveToPose.Result(response_code=0)
        elif skill_result.status == GoalStatus.STATUS_SUCCEEDED:
            goal_handle.succeed()
            self._logger.info("Skill executed successfully.")
            return FpmHuMoveToPose.Result(response_code=0)
        elif skill_result.status == GoalStatus.STATUS_CANCELED:
            goal_handle.canceled()
            self._logger.warn("Skill canceled.")
            return FpmHuMoveToPose.Result(response_code=0)
        elif skill_result.status == GoalStatus.STATUS_UNKNOWN:
            goal_handle.status == GoalStatus.STATUS_UNKNOWN
            self._logger.error("Skill status unknown.")
            return FpmHuMoveToPose.Result(response_code=0)


def get_action(node: Node):
    return HuMoveToPose(
        node=node,
        action_type=FpmHuMoveToPose,
        action_name=SKILL_NAME,
    )
