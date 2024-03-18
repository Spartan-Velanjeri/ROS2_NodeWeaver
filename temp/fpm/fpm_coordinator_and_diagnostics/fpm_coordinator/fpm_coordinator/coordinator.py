"""Coordinator module."""

import code
import traceback

import rclpy
from bautiro_ros_interfaces.action import StartFpmSkill
from fpm_coordinator.actions.drill_manually import SKILL_NAME as SN3
from fpm_coordinator.actions.drill_manually import get_action as GA3
from fpm_coordinator.actions.hu_move_to_pose import SKILL_NAME as SN5
from fpm_coordinator.actions.hu_move_to_pose import get_action as GA5
from fpm_coordinator.actions.interactive_control import SKILL_NAME as SN4
from fpm_coordinator.actions.interactive_control import get_action as GA4
from fpm_coordinator.actions.lift_move_absolute import SKILL_NAME as SN1
from fpm_coordinator.actions.lift_move_absolute import get_action as GA1
from fpm_coordinator.actions.lift_move_to_working_height import (
    SKILL_NAME as SN2,
)
from fpm_coordinator.actions.lift_move_to_working_height import (
    get_action as GA2,
)
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

NODE_NAME = "coordinator"
SKILL_NAMES = [SN1, SN2, SN3, SN4, SN5]


class Coordinator(Node):
    def __init__(self) -> None:
        super().__init__(NODE_NAME)

        self.bt_client = ActionClient(
            node=self, action_type=StartFpmSkill, action_name="fpm_skill_server"
        )

        self.logger = self.get_logger()

        self.logger.info("Creating coordinator actions.")
        # add actions
        GA1(node=self)
        GA2(node=self)
        GA3(node=self)
        GA4(node=self)
        GA5(node=self)

        # def wait_for_skill_server(self, result):
        #     server_ready = self._coordinator_node.bt_client.wait_for_server(
        #         timeout_sec=1
        #     )

        self._test_publisher = self.create_publisher(
            msg_type=String, topic="coordinator_topic", qos_profile=10
        )

        self._publisher_callback = self.create_timer(
            timer_period_sec=1, callback=self.test_pub_callback
        )

    def test_pub_callback(self):
        # self.logger.info("Timer.")
        msg = String()
        msg.data = "I Am Here!"
        self._test_publisher.publish(msg=msg)


def main():
    rclpy.init()

    try:
        coordinator = Coordinator()
        executor = MultiThreadedExecutor(4)
        executor.add_node(node=coordinator)
        try:
            executor.spin()

        except KeyboardInterrupt:
            coordinator.get_logger().fatal("Coordinator keyboard interrupt.")

        except BaseException as e:
            eString = "".join(
                traceback.format_exception(
                    etype=type(e), value=e, tb=e.__traceback__
                )
            )
            Node(node_name=NODE_NAME).get_logger().fatal(
                f"Coordinator. Unknown exception:{eString}."
            )

        coordinator.get_logger().fatal("Coordinator shuting down.")
        rclpy.shutdown()

    except Exception as e:
        eString = "".join(
            traceback.format_exception(
                etype=type(e), value=e, tb=e.__traceback__
            )
        )
        Node(node_name=NODE_NAME).get_logger().fatal(
            f"Coordinator unknown exception: {eString}."
        )
