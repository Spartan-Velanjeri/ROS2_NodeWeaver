from pathlib import Path
from typing import Any

import rclpy
from bautiro_ros_interfaces.action import (
    FpmPtuDrillOnCurrentPosition,
    StartFpmSkill,
)
from rclpy.action import ActionServer
from rclpy.action.server import ActionServer, ServerGoalHandle
from rclpy.node import Node

SKILL_NAME = Path(__file__).stem


class SkillActionServer(ActionServer):
    def __init__(self, node: Node, action_type: Any, action_name: Any) -> None:
        print("init called")
        super().__init__(
            node=node,
            action_type=action_type,
            action_name=action_name,
            execute_callback=self.action_callback,
        )

        self._coordinator_node = node

        print("init done")

    def action_callback(
        self, goal_handle: ServerGoalHandle
    ) -> FpmPtuDrillOnCurrentPosition.Result:
        payload = goal_handle.request.working_height

        print("Here 1")
        # call benaviout tree skill server
        msg = StartFpmSkill.Goal()

        print("HEre2")
        msg.skill_name = ""
        msg.in_payload = ""

        server_ready = self._coordinator_node.bt_client.wait_for_server(
            timeout_sec=1
        )

        if not server_ready:
            goal_handle.abort()

        future = self.send_goal_async(msg)


def get_action(node: Node) -> ActionServer:
    """Resturns action server for the skill.

    Args:
        node (Node): ROS Node instance to which action should be assigned.

    Returns:
        ActionServer: Action server for the skill.
    """
    return SkillActionServer(
        node=node,
        action_type=FpmPtuDrillOnCurrentPosition,
        action_name=SKILL_NAME,
    )
