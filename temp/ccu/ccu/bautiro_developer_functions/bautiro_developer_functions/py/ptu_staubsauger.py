# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from rclpy.node import Node
from std_srvs.srv import Trigger
from bautiro_developer_functions.core.call_srv import call_service_str
from bautiro_developer_functions.core.lib import MIN, MAX, STEP, limit


def ptu_an(node: Node) -> str:

    # call_service_str(node, Trigger, '/ptu_srv', Trigger.Request())
    return 'Not implemented (ptu_an)'


def ptu_aus(node: Node) -> str:

    # call_service_str(node, Trigger, '/ptu_srv', Trigger.Request())
    return 'Not implemented (ptu_aus)'


PTU_ANSTEUERN = {
    'drehzahl':       {MIN:  0.0, MAX: 8500.0, STEP: 100.0},
}


def ptu_ansteuern(node: Node,
                  drehzahl: float,
                  rechts_drehend: bool) -> str:

    d = limit(drehzahl,       'drehzahl',       PTU_ANSTEUERN)
    r = rechts_drehend

    # call_service_str(node, Trigger, '/ptu_srv', Trigger.Request())
    return 'Not implemented (ptu_ansteuern)'


###################################################################################################
#                                                                                                 #
#  Staubsauger                                                                                    #
#                                                                                                 #
###################################################################################################


def staubsauger_an(node: Node) -> str:
    # TODO
    # call_service_str(node, Trigger, '/staubsauger_srv', Trigger.Request())
    return 'Not implemented (staubsauger_an)'


def staubsauger_aus(node: Node) -> str:
    # TODO
    # call_service_str(node, Trigger, '/staubsauger_srv', Trigger.Request())
    return 'Not implemented (staubsauger_aus)'
