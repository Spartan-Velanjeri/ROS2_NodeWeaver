#! /usr/bin/env python3
# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

"""Node that checks for proper use of sim_time."""

import time
from functools import partial
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters


class CheckNode(Node):
    """
    Node that checks use_sim_time.

    It will periodically get a list of all node names and query their parameter service. From
    this it will skip a number of known-irrelevant nodes or nodes that should never use sim_time.
    The rest it will query via their parameter service.

    Once it has checked a node, it will be added either to an "OK" or a "NOT OK" list and
    won't be checked again in the future. It will also maintain an "unresponsive" list of nodes
    it couldn't check. Problems are reported once every 10 seconds.

    Once all relevant nodes OK nodes and there has been no change to the list for 30 seconds, it
    will terminate.
    """

    IGNORE_ARG = "ignore"
    # terminate this many seconds after no change has been seen
    NO_CHANGE_TIMEOUT = 60

    def __init__(self):
        super().__init__("check_sim_time_node")
        self.check_timer = self.create_timer(1, self.check_sim_time_cb)
        self.report_timer = self.create_timer(10, self.report_cb)
        self.seen = []
        self.ok = set()
        self.nok = set()
        self.unresponsive = set()
        self.log = rclpy.logging.get_logger(self.get_name())
        self.ignore_nodes = self.declare_parameter(
            name=self.IGNORE_ARG,
            value=["/robot_state_publisher", "/ignition_ros_control"],
        )
        self.last_change = 0

    def completion_cb(self, client, node_name: str, future: rclpy.task.Future):
        """Check result for match with expectations. Also clean svc."""
        r = future.result()
        if r is None or not isinstance(r, GetParameters.Response):
            self.log.warn(f"GetParameter service returned inappropriate response: {r}")
        else:
            # remember last time for inactivity check
            self.last_change = time.time()
            if r.values[0].bool_value:
                self.ok.add(node_name)
                self.log.info(f"{node_name} use_sim_time is ok")
            else:
                self.nok.add(node_name)
        # clean up
        client.destroy()

    def report_cb(self):
        """Report the check status."""
        # skip if no data, yet
        if len(self.nok) > 0:
            self.log.error(f"use_sim_time not ok: {self.nok}")
        if len(self.unresponsive) > 0:
            self.log.warn(f"Unresponsive: {self.unresponsive}")

    def _can_ignore(self, name):
        ignore_names = self.get_parameter(self.IGNORE_ARG).value
        return (
            name == self.get_fully_qualified_name()
            or name in self.seen
            or name in ignore_names
            or "ros2cli" in name
            or "/transform_listener_impl" in name
            or "rqt_gui" in name
            or "costmaplocal" in name
            or "costmapglobal" in name
            or "rclcpp_node" in name
            or "check_sim_time" in name
        )

    def check_sim_time_cb(self):
        """Query all nodes for use_sim_time param value."""
        nodes = self.get_node_names_and_namespaces()
        req = GetParameters.Request()
        req.names = ["use_sim_time"]
        full_names = [f"{ns}{node_name}" for node_name, ns in nodes]

        relevant_nodes = [fname for fname in full_names if not self._can_ignore(fname)]
        self.log.debug(f"Nodes to check {relevant_nodes}")
        if (
            len(relevant_nodes) == 0
            and time.time() - self.last_change > self.NO_CHANGE_TIMEOUT
        ):
            self.log.info("No changes to nodes, terminating use_sim_time check")
            self.check_timer.destroy()
            self.report_timer.destroy()
            self.destroy_node()
            raise SystemExit

        for full_node_name in relevant_nodes:
            svc_name = f"{full_node_name}/get_parameters"

            client = self.create_client(GetParameters, svc_name)
            self.log.debug(f"Check {svc_name} readiness")
            if not client.service_is_ready():
                self.log.debug(f"Skipping {svc_name}")
                self.unresponsive.add(full_node_name)
                client.destroy()
                continue
            else:
                self.unresponsive.discard(full_node_name)
                self.seen.append(full_node_name)
            self.log.debug(f"Querying {svc_name}")
            future = client.call_async(req)
            future.add_done_callback(
                partial(self.completion_cb, client, full_node_name)
            )

        # clean up not ok list, removing nodes that are no longer present
        keep = set()
        for node_name in self.nok:
            if node_name in full_names:
                keep.add(node_name)
        self.nok = keep


def run(args=None):
    """Create node and spin it."""
    rclpy.init(args=args)
    n = CheckNode()
    try:
        rclpy.spin(n)
    except SystemExit:
        pass
    rclpy.shutdown()
    return 0
