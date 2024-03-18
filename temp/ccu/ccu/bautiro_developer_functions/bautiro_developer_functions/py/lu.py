# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from rclpy.node import Node
from std_srvs.srv import Trigger
from ccu_interfaces.srv import GenericPrim
from bautiro_developer_functions.core.call_srv import call_service_str


def start_feinlokalisierung(node: Node) -> str:

    # call_service_str(node, Trigger, '/feinlokalisierung', Trigger.Request())
    return 'Not implemented (start_feinlokalisierung)'
