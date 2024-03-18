import multiprocessing
from threading import Thread

import pytest
import rclpy
import ros2cli
from bautiro_ros_interfaces.action import FpmLiftMoveAbsolute, StartFpmSkill
from fpm_coordinator.coordinator import SKILL_NAMES, Coordinator
from rclpy.action import ActionClient, ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

SKILL_SERVER_NAME = "fpm_skill_server"


class BTDummySkillServer(Node):
    def __init__(self) -> None:
        super().__init__("BTDummyNode")

        self._skill_server = ActionServer(
            self,
            action_type=StartFpmSkill,
            action_name=SKILL_SERVER_NAME,
            execute_callback=self.skill_callback,
        )

    def skill_callback(
        self, goal_handle: ServerGoalHandle
    ) -> StartFpmSkill.Result:
        skill_name = goal_handle.request.skill_name
        lift_setpoint_position = goal_handle.request.lift_setpoint_position
        robot_arm_pose = goal_handle.request.robot_arm_pose
        platform_ceiling_distance = (
            goal_handle.request.platform_ceiling_distance
        )

        if skill_name == "lift_move_absolute":
            # requesting absolute lift move
            print("Skill <lift_move_absolute> requested.")
            assert lift_setpoint_position
            assert not robot_arm_pose
            assert not platform_ceiling_distance
        elif skill_name == "lift_move_to_working_height":
            assert not lift_setpoint_position
            assert not robot_arm_pose
            assert platform_ceiling_distance


# @pytest.fixture(scope="module")
# def launch_dummy_nodes() -> MultiThreadedExecutor:
#     print("Starting dummy nodes...")
#     rclpy.init()
#     bt_dummy_skill_server = BTDummySkillServer()
#     executor = MultiThreadedExecutor(4)
#     executor.add_node(node=bt_dummy_skill_server)
#     coordinator = Coordinator()
#     executor.add_node(node=coordinator)
#     dummy_nodes_thread = Thread(target=executor.spin, daemon=True)
#     dummy_nodes_thread.start()

#     return executor


# @pytest.fixture(scope="session", autouse=True)
# def cleanup(launch_dummy_nodes: MultiThreadedExecutor):
#     """Cleanup function for closing the thread."""
#     launch_dummy_nodes.shutdown()


# @pytest.fixture(scope="session")
# def stop_driver():
#     print("Stopping driver...")

#     print("Driver stopped.")


# def generate_test_description() -> LaunchDescription:
#     """Description of the test.

#     Returns:
#         LaunchDescription: Test description.
#     """
#     lift_driver = Node(package="fpm_ctrl_x_driver", executable="lift_driver")

#     return LaunchDescription([lift_driver, ReadyToTest])


# @pytest.mark.rostest
# def test_lift_status():
#     print("Starting lift status test...")

#     client = Node(node_name="pytest").create_client(srv_type=,srv_name=)

#     print("End lift test.")


def test_skill_lift_move_absolute():
    """_summary_"""
    testNode = Node(node_name="test_node")
    logger = testNode.get_logger()
    logger.info("-------test_skill_lift_move_absolute--------------")

    logger.info("Creating test action client.")
    ac: ActionClient = ActionClient(
        node=testNode,
        action_type=FpmLiftMoveAbsolute,
        action_name="lift_move_absolute",
    )
    testNode.test_client = ac

    import time

    goal = FpmLiftMoveAbsolute.Goal()
    if not testNode.test_client.wait_for_server(timeout_sec=2):
        assert False, "Coordinator server not ready."
    else:
        logger.info("Action server lift_move_absolute ready.")

    goal.lift_setpoint_position = 1.0
    future = testNode.test_client.send_goal_async(goal=goal)

    time.sleep(3)
    print(future.result())
    assert False
    counter = 0
    # while not future.done():
    #     import time

    #     time.sleep(0.2)
    #     print("BLA")
    #     if counter > 100:
    #         break
    #     counter += 1

    # assert future.result()
