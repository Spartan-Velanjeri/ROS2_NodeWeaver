# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from typing import Any
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.client import Client
from bautiro_developer_functions.core.node_delegator import NodeDelegator
from bautiro_developer_functions.core.call_action_by_client import (call_fire_and_forget,
                                                                    call_str,
                                                                    call_result)
from bautiro_developer_functions.core.call_action_by_node import (call_action_fire_and_forget,
                                                                  call_action_fire_and_forget_bool,
                                                                  call_action_str,
                                                                  call_action_result)
from bautiro_developer_functions.core.call_srv import call_service_response, call_service_str


class Caller(NodeDelegator):

    def __init__(self, node: Node) -> None:
        super().__init__(node)

    def call_fire_and_forget(self, ac_type: type,
                             ac_srv_name: str,
                             goal: Any,
                             *,
                             ac: ActionClient) -> str:
        return call_fire_and_forget(ac_type, ac_srv_name, goal, ac=ac)

    def call_str(self, ac_type: type,
                 ac_srv_name: str,
                 goal: Any,
                 timeout: float = None,
                 *,
                 ac: ActionClient) -> str:
        return call_str(ac_type, ac_srv_name, goal, timeout, ac=ac)

    def call_result(self, ac_type: type,
                    ac_srv_name: str,
                    goal: Any,
                    timeout: float = None,
                    *,
                    ac: ActionClient) -> Any:
        return call_result(ac_type, ac_srv_name, goal, timeout, ac=ac)

    def call_action_fire_and_forget(self, ac_type: type,
                                    ac_srv_name: str,
                                    goal: Any) -> str:
        return call_action_fire_and_forget(ac_type, ac_srv_name, goal, node=self.node)

    def call_action_fire_and_forget_bool(self, ac_type: type,
                                         ac_srv_name: str,
                                         goal: Any) -> bool:
        return call_action_fire_and_forget_bool(ac_type, ac_srv_name, goal, node=self.node)

    def call_action_str(self, ac_type: type,
                        ac_srv_name: str,
                        goal: Any,
                        timeout: float = None) -> str:
        return call_action_str(ac_type, ac_srv_name, goal, timeout, node=self.node)

    def call_action_result(self, ac_type: type,
                           ac_srv_name: str,
                           goal: Any,
                           timeout: float = None) -> str:
        return call_action_result(ac_type, ac_srv_name, goal, timeout, self.node)

    def call_service_response(self, srv_type: type,
                              srv_name: str,
                              request: Any,
                              *,
                              client: Client = None) -> Any:
        return call_service_response(self.node, srv_type, srv_name, request, client=client)

    def call_service_str(self, srv_type: type,
                         srv_name: str,
                         request: Any,
                         *,
                         client: Client = None) -> str:
        return call_service_str(self.node, srv_type, srv_name, request, client=client)
