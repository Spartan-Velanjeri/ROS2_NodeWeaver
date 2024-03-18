#! /usr/bin/env python3
# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

"""Node that checks managed node status."""

from threading import Event
import time
from functools import partial
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from rcl_interfaces.msg import ParameterDescriptor


def terminate_cb():
    raise SystemExit


class CheckLifecycleManagersNode(Node):
    """
    Module that checks managed node status and exits when all are running.

    It will periodically check running nav2_lifecycle_manager's on their is_active service
    and if all are active, it will exit.
    """

    LC_NAMES = "lifecycle_managers"
    NO_CHANGE_TIMEOUT = 30

    def __init__(self, finished_cb=terminate_cb):
        """
        Create node that checks lifecycle manager status.

        :param: finished_cb function to call when status checking is complete
        """
        super().__init__("check_lifecycle_managers_node")
        self.check_timer = self.create_timer(1, self.check_cb)
        self.report_timer = self.create_timer(10, self.report_cb)
        self.seen = []
        self.ok = set()
        self.nok = set()
        self.log = rclpy.logging.get_logger(self.get_name())
        self.lc_names_param = self.declare_parameter(
            name=self.LC_NAMES,
            descriptor=ParameterDescriptor(
                name=self.LC_NAMES,
                type=9,
                description="List of qualified lifecycle manager names.",
            ),
        )
        self.lc_names = []
        self.last_change = time.time()
        self.finished_cb = finished_cb

    def _completion_cb(self, client, node_name: str, future: rclpy.task.Future):
        """Check result for match with expectations. Also clean svc."""
        r = future.result()
        if r is None or not isinstance(r, Trigger.Response):
            self.log.warn(f"Trigger service returned inappropriate response: {r}")
        else:
            # remember last time for inactivity check
            self.last_change = time.time()
            if r.success:
                self.nok.discard(node_name)
                self.ok.add(node_name)
                self.log.info(f"{node_name} is active")
            else:
                self.nok.add(node_name)
        # clean up
        client.destroy()

    def report_cb(self):
        """Report the check status."""
        # skip if no data, yet
        if len(self.nok) > 0:
            self.log.error(f"Not yet active: {self.nok}")

    def _can_ignore(self, name):
        ignore_names = self.get_parameter(self.IGNORE_ARG).value
        return name in self.ok or name in ignore_names

    def check_cb(self):
        """Query all is_active services."""
        self.lc_names = (
            self.get_parameter(self.LC_NAMES).get_parameter_value().string_array_value
        )
        self.log.info("%s" % self.lc_names)
        services_to_check = []
        if len(self.lc_names) > 0:
            services_to_check = [f"{name}/is_active" for name in self.lc_names]
        else:
            # get list of services matching is_active services
            nodes = self.get_node_names_and_namespaces()
            for node_name, ns in nodes:
                services = self.get_service_names_and_types_by_node(node_name, ns)
                for service_name, service_types in services:
                    self.log.debug(f"Considering {service_name}{service_types}")
                    if (
                        service_name.endswith("/is_active")
                        and "std_srvs/srv/Trigger" in service_types
                        and service_name not in self.ok
                    ):
                        services_to_check.append(service_name)
        self.log.debug(f"Services to check {services_to_check}")

        # terminate if we're done
        # if we got an explicit list of managers to check, check that it is present
        if len(self.lc_names) > 0 and len(self.lc_names) == len(self.ok):
            self.log.info("Configured services active, terminating")
            self.destroy_node()
            raise SystemExit
        elif (
            len(self.ok) > 0
            and len(services_to_check) == 0
            and time.time() < self.last_change > self.NO_CHANGE_TIMEOUT
        ):
            self.log.info("All known services active, terminating.")
            self.finished_cb()

        # query services for status
        req = Trigger.Request()
        for svc_name in services_to_check:
            client = self.create_client(Trigger, svc_name)
            self.log.debug(f"Check {svc_name} readiness")
            if not client.service_is_ready():
                self.log.debug(f"Skipping {svc_name}")
                client.destroy()
                continue
            self.log.debug(f"Querying {svc_name}")
            future = client.call_async(req)
            future.add_done_callback(partial(self._completion_cb, client, svc_name))


def run(args=None):
    """Create node and spin it."""
    rclpy.init(args=args)
    n = CheckLifecycleManagersNode()
    try:
        rclpy.spin(n)
    except SystemExit:
        n.destroy()
    rclpy.shutdown()
    return 0


def wait_for_active(timeout=30.0):
    """
    Check lifecycle managers are active.

    Runs the CheckLifecycleManagersNode until it is finished, or
    until the timeout.
    :return: true when check is successful, false on timeout
    """
    ev = Event()
    n = CheckLifecycleManagersNode(finished_cb=ev.set())
    # spin until complete or until timeout
    start = time.time()
    while not ev.is_set():
        time.sleep(0.1)
        rclpy.spin_once(n)
        if time.time() - start > timeout:
            break

    return ev.is_set()
