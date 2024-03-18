# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from time import sleep
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, Vector3
from bautiro_ros_interfaces.srv import GetClusterPosition
from bautiro_developer_functions.core.call_srv import call_service_str
from bautiro_developer_functions.core.lib import MIN, MAX, STEP, limit


###################################################################################################
#                                                                                                 #
#  Manuelles Homing                                                                               #
#                                                                                                 #
###################################################################################################


def manuelles_homing(node: Node) -> str:
    # TODO
    return 'Not implemented (manuelles_homing)'


###################################################################################################
#                                                                                                 #
#  Grob-Pose anfahren                                                                             #
#                                                                                                 #
###################################################################################################


def next_grobpose_anfahren(node: Node) -> str:
    r = _get_cluster_position(node, '/get_cluster_position')
    next_pose: Pose = r.cluster_positions[0]
    # TODO call_service_str(node, TYP, '/move_to_pose', TYP.Request(target_pose=next_pose))
    return 'Not implemented (next_grobpose_anfahren)'


def grobpose_n_anfahren(node: Node, cluster_index: int = 0) -> str:
    # cluster_index       RPM moves to ...
    #       0        -->   cluster <next> - based on autonomous drilling
    #      <n>       -->   cluster <n>
    r = _get_cluster_position(node, '/all_cluster_positions')
    pose_n: Pose = r.cluster_positions[cluster_index]
    # TODO call_service_str(node, TYP, '/move_to_pose', TYP.Request(target_pose=pose_n))
    return 'Not implemented (grobpose_n_anfahren)'


def _get_cluster_position(node: Node, srv_name: str) -> GetClusterPosition.Response:
    return call_service_str(node, GetClusterPosition, srv_name, GetClusterPosition.Request())

###################################################################################################
#                                                                                                 #
#  freies fahren                                                                                  #
#                                                                                                 #
###################################################################################################


def rpm_1_sec_geradeaus_fahren(node: Node) -> str:
    return rpm_1_sec_in_richtung_fahren(node, translation=1.0, rotation=0.0, speed=1.0)


RPM_1_SEC_IN_RICHTUNG_FAHREN = {
    'translation': {MIN: -1.0, MAX: 1.0},
    'rotation':    {MIN: -1.0, MAX: 1.0},
    'speed':       {MIN:  0.0, MAX: 1.0}
}


def rpm_1_sec_in_richtung_fahren(
        node: Node,
        translation: float = 1.0,
        rotation: float = 0.0,
        speed: float = 1.0) -> str:
    pub = node.create_publisher(Twist, '/rpm_velocity_controller/cmd_vel_unstamped', 10)

    t = limit(translation, 'translation', RPM_1_SEC_IN_RICHTUNG_FAHREN)
    r = limit(rotation,    'rotation',    RPM_1_SEC_IN_RICHTUNG_FAHREN)
    s = limit(speed,       'speed',       RPM_1_SEC_IN_RICHTUNG_FAHREN)

    def _do_pub(node: Node) -> None:
        pub.publish(Twist(linear=Vector3(x=t * max(s, 0.8)), angular=Vector3(z=r)))

    # erzeuge einen Timer, der alle 0.1 Sekunden eine Fahrt-in-Richtungs-Twist published
    timer = node.create_timer(timer_period_sec=0.1, callback=_do_pub)
    duration = 1.0
    sleep(duration)
    # Timer nach 1 Sekunde wieder abräumen
    flag = node.destroy_timer(timer)
    flag = node.destroy_publisher(pub) and flag
    return f"Moved RPM for {duration} seconds. (Timer & Publisher disposed: {flag})"
