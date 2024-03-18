# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from typing import Any
from threading import Event
from rclpy.node import Node
from rclpy.client import Client
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup as CbG

from bautiro_developer_functions.core.node_delegator import NodeDelegator


_TIMEOUT_CONNECT_TO_SERVICE = 3.0   # 5 sec
_TIMEOUT_RESPONSE_REQUEST = 10.0   # 5 sec


def call_service_str(node: Node,
                     srv_type: type,
                     srv_name: str,
                     request: Any,
                     *,
                     client: Client = None) -> str:
    nd = NodeDelegator(node)
    resp = __call_service_response(nd, client, srv_type, srv_name, request)
    if isinstance(resp, str):  # Error case
        return resp
    if resp is None:
        return nd.warn(f'No Response received from {srv_name}')
    return nd.debug(f'Response received from {srv_name}: {repr(resp)}')


def call_service_response(node: Node,
                          srv_type: type,
                          srv_name: str,
                          request: Any,
                          *,
                          client: Client = None) -> Any:
    nd = NodeDelegator(node)
    resp = __call_service_response(nd, client, srv_type, srv_name, request)
    if isinstance(resp, str):  # Error case
        return None
    if resp is None:
        nd.warn(f'No Response received from {srv_name}')
    nd.debug(f'Response received from {srv_name}: {repr(resp)}')
    return resp  # can be None


def __call_service_response(nd: NodeDelegator,
                            cl: Client,
                            srv_type: type,
                            srv_name: str,
                            request: Any) -> Any:  # SERVICE.RESPONSE_TYPE
    created = False
    if cl is None:
        cl = nd.node.create_client(srv_type, srv_name, callback_group=CbG())
        created = True
    nd.debug(f'Wait for service-server "{srv_name}" ...')
    if not cl.wait_for_service(timeout_sec=_TIMEOUT_CONNECT_TO_SERVICE):
        if created:
            cl.destroy()
        return nd.warn(f'Cannot connect to service "{srv_name}" ')

    nd.log(f'Send service-request {srv_type.__qualname__} to server {srv_name} ...')

    # Future 1 - short running: Goal accept / reject
    f_done = Event()
    future = cl.call_async(request)
    future.add_done_callback(lambda future: f_done.set())
    if not f_done.wait(timeout=_TIMEOUT_RESPONSE_REQUEST):
        if created:
            cl.destroy()
        return nd.warn(f'No response in (10s) from "{srv_name}"')

    response = future.result()
    if created:
        cl.destroy()
    return response  # can be None
