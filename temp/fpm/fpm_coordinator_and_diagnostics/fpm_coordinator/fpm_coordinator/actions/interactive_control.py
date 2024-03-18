import code
import readline
import time
from pathlib import Path
from typing import Any

from bautiro_ros_interfaces.action import FpmInteractiveControl, StartFpmSkill
from bautiro_ros_interfaces.msg import ResponseCode
from rclpy.action import ActionClient, ActionServer
from rclpy.action.server import ActionServer, GoalStatus, ServerGoalHandle
from rclpy.node import Node

SKILL_NAME = Path(__file__).stem


class InteractiveControl(ActionServer):
    """Action server for interactive control program."""

    def __init__(self, node: Node, action_type: Any, action_name: Any) -> None:
        """Initialization of the action server

        Args:
            node (Node): Node to which belongs the action server.
            action_type (Any): Data type of the action server.
            action_name (Any): Name of the action server
        """
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
    ) -> FpmInteractiveControl.Result:
        """Action server callback that implements functionality of the server.

        Args:
            goal_handle (ServerGoalHandle): Handle to the goal data structure.

        Returns:
            FpmLiftMoveToWorkingHeight.Result: Result of the action.
        """
        self._logger.info(
            f'Skill "{SKILL_NAME}" called with payload "{goal_handle.request}".'
        )
        import pynput
        from pynput import keyboard

        def on_press(key):
            try:
                print("alphanumeric key {0} pressed".format(key.char))
            except AttributeError:
                print("special key {0} pressed".format(key))

        def on_release(key):
            print("{0} released".format(key))
            if key == keyboard.Key.esc:
                # Stop listener
                return False

        # Collect events until released
        with keyboard.Listener(
            on_press=on_press, on_release=on_release
        ) as listener:
            listener.join()

        selected_key = 0
        while selected_key != chr(27):
            print("Interactive program started. Select one of the options:")
            print("/t1 - Lift programs")
            print("/t2 - Robot programs")
            print("/t3 - drill programs")
            print("/t4 - automatic programs")
            print("Esc. - abort main program")

        return FpmInteractiveControl.Result(response_code=0)

        # # call behavior tree skill server
        # msg = StartFpmSkill.Goal()

        # msg.skill_name = "fpm_drill_manually.xml"
        # msg.platform_ceiling_distance = (
        #     goal_handle.request.platform_ceiling_distance
        # )

        # # check is the skill server ready
        # server_ready = self._skill_client.wait_for_server(timeout_sec=1)
        # if not server_ready:
        #     goal_handle.abort()
        #     self._logger.error("Skill server time-out. Aborting action.")
        #     return FpmInteractiveControl.Result(response_code=0)

        # future = self._skill_client.send_goal_async(msg)

        # while not future.done():
        #     # wait to accept or decline the request
        #     time.sleep(0.2)

        # skill_result = future.result()

        # while skill_result.status in [
        #     GoalStatus.STATUS_EXECUTING,
        #     GoalStatus.STATUS_CANCELING,
        # ]:
        #     self._logger.info(f"{SKILL_NAME}: skill is executing.")
        #     time.sleep(2)

        # if skill_result.status == GoalStatus.STATUS_ABORTED:
        #     goal_handle.abort()
        #     self._logger.error("Skill server aborted the skill.")
        #     return FpmInteractiveControl.Result(response_code=0)
        # elif skill_result.status == GoalStatus.STATUS_SUCCEEDED:
        #     goal_handle.succeed()
        #     self._logger.info("Skill executed successfully.")
        #     return FpmInteractiveControl.Result(response_code=0)
        # elif skill_result.status == GoalStatus.STATUS_CANCELED:
        #     goal_handle.canceled()
        #     self._logger.warn("Skill canceled.")
        #     return FpmInteractiveControl.Result(response_code=0)
        # elif skill_result.status == GoalStatus.STATUS_UNKNOWN:
        #     goal_handle.status == GoalStatus.STATUS_UNKNOWN
        #     self._logger.error("Skill status unknown.")
        #     return FpmInteractiveControl.Result(response_code=0)


def get_action(node: Node) -> ActionServer:
    """Returns instance of the above action server.

    Args:
        node (Node): ROS node to which the action should be assigned to.

    Returns:
        ActionServer: DrillManually action server.
    """
    return InteractiveControl(
        node=node,
        action_type=FpmInteractiveControl,
        action_name=SKILL_NAME,
    )
