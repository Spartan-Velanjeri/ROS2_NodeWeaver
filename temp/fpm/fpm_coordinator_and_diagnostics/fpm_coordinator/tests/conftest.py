"""Test configurations.
"""
import multiprocessing
from threading import Thread

import pytest
import rclpy
import ros2cli
from bautiro_ros_interfaces.action import FpmLiftMoveAbsolute, StartFpmSkill
from fpm_coordinator.coordinator import SKILL_NAMES, Coordinator
from rclpy.action import ActionClient, ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node

SKILL_SERVER_NAME = "fpm_skill_server"
rclpy.init()
EXECUTOR = MultiThreadedExecutor(10)
PROCESS: multiprocessing.Process


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
        assert False
        if skill_name == "lift_move_absolute":
            # requesting absolute lift move
            print("Skill <lift_move_absolute> requested.")
            assert lift_setpoint_position
            assert not robot_arm_pose
            assert not platform_ceiling_distance

            result = StartFpmSkill.Result()
            result.test = 1
            return result

        elif skill_name == "lift_move_to_working_height":
            assert not lift_setpoint_position
            assert not robot_arm_pose
            assert platform_ceiling_distance


def pytest_configure(config):  # pylint: disable=unused-argument
    """Pre-configuration hook that runs before tests.

    Args:
        _config (pytest.Config): Pytest config.
    """
    global PROCESS
    logger = Node(node_name="pytest_configure").get_logger()
    logger.info("********Configuration of tests.*******")
    logger.info("Launching dummy nodes...")

    # executor = MultiThreadedExecutor(4)
    EXECUTOR.add_node(node=BTDummySkillServer())
    EXECUTOR.add_node(node=Coordinator())
    PROCESS = multiprocessing.Process(
        target=EXECUTOR.spin(),
    )
    PROCESS.start()

    import time

    r


def pytest_unconfigure(config):  # pylint: disable=unused-argument
    """Post-configuration hook that runs after tests.

    Args:
        _config (pytest.Config): Pytest config.
    """
    global PROCESS
    logger = Node(node_name="pytest_configure").get_logger()
    # delete test database
    logger.info("********Cleaning hook.*******")
    nodes = EXECUTOR.get_nodes()
    logger.info(f"Is process alive {PROCESS.is_alive()}")
    for node in nodes:
        logger.info(f"Destroying node {node.get_name()}")
        node.destroy_node()

    PROCESS.kill()
