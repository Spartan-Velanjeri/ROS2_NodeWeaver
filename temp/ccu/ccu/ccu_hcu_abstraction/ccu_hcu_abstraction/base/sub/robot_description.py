# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from time import sleep, time
from typing import List, Dict

from xml.etree import ElementTree
from xml.etree.ElementTree import Element

from rclpy.node import Node
from rclpy.time import Time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup as CbG
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterValue
from geometry_msgs.msg import Transform
from tf2_ros import Buffer, TransformListener

from ccu_grpc.base_layer_service_pb2 import (BautiroUrdfLink, BautiroUrdfVisualMesh,
                                             BautiroDescriptionMessage)
from bautiro_developer_functions.core.node_delegator import NodeDelegator
from ccu_util.util import calc_md5, rpy_to_matrix4x4, tf_to_matrix4x4

###################################################################################################
#
#   intermediate model  (see .proto)
#
###################################################################################################


class CBautiroUrdfLink():
    def _init_(self):
        self.name: str = None
        self.parent: str = None
        self.visuals: List[BautiroUrdfVisualMesh] = []

###################################################################################################
#
#   Node to access ("tf") and (Parameter["robot_description"])
#
###################################################################################################


class RobotDescription(NodeDelegator):

    def __init__(self, node: Node):
        super().__init__(node)
        self.__bull = None  # Bautiro Urdf Link List
        self.transform_listener = TransformListener(Buffer(), node)
        self.buffer = self.transform_listener.buffer
        self.cl = node.create_client(GetParameters, '/robot_state_publisher/get_parameters',
                                     callback_group=CbG())

    def get_bautiro_description(self) -> BautiroDescriptionMessage:
        """Return `None` or complete Robot-Description with momentary transforms."""
        links = []
        if tms := self._get_tms_loop():
            for bul in self._bautiro_urdf_link_list:
                tm: Transform = tms[bul]
                links.append(BautiroUrdfLink(name=bul.name,
                                             parent=bul.parent,
                                             visuals=bul.visuals,
                                             transform_relative_to_parent=tf_to_matrix4x4(tm)))
            return BautiroDescriptionMessage(links=links)
        return BautiroDescriptionMessage()

    @property
    def _bautiro_urdf_link_list(self) -> List[CBautiroUrdfLink]:
        if self.__bull is None:
            self.__bull = self._build_bautiro_urdf_link_list()
        return self.__bull

    def _get_root_element(self) -> Element:
        if urdf := self._get_str():
            return ElementTree.fromstring(urdf)

    def _get_str(self) -> str:
        resp: GetParameters.Response = self._get_parameter()
        if resp and resp.values:
            if len(resp.values) > 1:
                print("Vorsicht!")
            value: ParameterValue = resp.values[0]
            return value.string_value

    def _get_parameter(self) -> GetParameters.Response:
        if not self.cl.wait_for_service(timeout_sec=3.0):
            self.info("Srv not available: '/robot_state_publisher/get_parameters'")
            return None
        return self.cl.call(GetParameters.Request(names=['robot_description']))

    def _get_tms_loop(self) -> Dict[CBautiroUrdfLink, Transform]:  # key-value-pairs from TF-Buffer
        start_time = time()
        while time() - start_time < 5:
            if tms := self._get_tms():
                return tms
            sleep(0.25)

    def _get_tms(self) -> Dict[CBautiroUrdfLink, Transform]:
        tms = {}
        for bul in self._bautiro_urdf_link_list:
            if bul.parent is None:
                tms[bul] = None  # for top-level-links without parent
            else:
                if tm := self._get_tm(self, bul.parent, bul.name):
                    tms[bul] = tm

        if len(tms) == len(self._bautiro_urdf_link_list):
            return tms

    def _get_tm(self, parent: str, child: str) -> Transform:
        """Return `tm` for given child->parent or `None` when Buffer still too empty."""
        try:
            return self.buffer.lookup_transform(parent, child, Time()).transform
        except Exception as e:
            self.info(f'Exception in tf monitor: {e}')  # Happens often

    def _build_bautiro_urdf_link_list(self) -> List[CBautiroUrdfLink]:
        bull = []
        if root := self._get_root_element():
            all_joints = root.findall('.//joint')
            for link in root.findall('.//link'):
                bull.append(CBautiroUrdfLink(name=(name := link.attrib.get('name')),
                                             parent=self._get_parent(name, all_joints),
                                             visuals=self._get_meshes(link)))
        return bull

    def _get_parent(self, child_name: str, all_joints: List[Element]) -> str:
        for j in all_joints:
            if child_name == j.find("child").attrib.get('link'):
                return j.find("parent").attrib.get('link')

    def _get_meshes(self, link: Element) -> List[BautiroUrdfVisualMesh]:
        result = []
        for visual in link.findall('./visual'):

            if (mesh := visual.find('./geometry/mesh')) is not None:
                if file := mesh.attrib.get('filename', None):
                    f = file.replace('file://', '')
                    m = calc_md5(f)
            if (origin := visual.find('./origin')) is not None:
                if f:
                    o = rpy_to_matrix4x4(origin.attrib.get('xyz', None),
                                         origin.attrib.get('rpy', None))
            if f:
                result.append(BautiroUrdfVisualMesh(mesh_file_path=f,
                                                    mesh_file_md5_hash=m,
                                                    offset=o))
        return result
