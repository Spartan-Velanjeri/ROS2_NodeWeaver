# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.


from typing import List, Tuple
from xml.etree import ElementTree
from xml.etree.ElementTree import Element

from ccu_bautiro.ccu_bautiro import CWorkPlan
from ccu_bautiro.Jobs import CJob, CDrillJob, CMeasureJob, CMoveJob
from ccu_bautiro.NodeTree import CDrillHole, CDrillMask, CFastener, CNode
from ccu_bautiro.Types import CMarker, CMatrix
from ccu_dataservice.conf import Conf
from ccu_dataservice.eutil import ResourceHandler

STRING_DEFAULT = ''
FLOAT_DEFAULT = 0.0

Q = Conf.FQN_SEPERATOR


def get_float(value: Element) -> float:
    """Return float Element content of an XML Element."""
    if isinstance(value, Element):
        value = value.text
    if isinstance(value, str) and len(value) > 0:
        return float(value)
    return FLOAT_DEFAULT


def get_string(value: any) -> str:
    """Read string content of XML Element."""
    if isinstance(value, Element):
        value = value.text
    if isinstance(value, str) and len(value) > 0:
        return value
    return STRING_DEFAULT


def get_attrib(key: str, element: Element, replacement_value: str = None) -> str:
    """Return attrib value from XML element, checks existence. replacement default optional."""
    if key in element.attrib:
        value = element.attrib[key]
        if value:
            return value
    if replacement_value:
        return replacement_value
    return STRING_DEFAULT


MM2M = 0.001  # MilliMeter-to-Meter conversion


def conv_tml(container_xml_element: Element, tm: CMatrix) -> CMatrix:
    e = container_xml_element.find('BaseObjectMl')
    return _convert_tm(e, tm)


def conv_tmg(container_xml_element: Element, tm: CMatrix) -> CMatrix:
    e = container_xml_element.find('BaseObjectMg')
    return _convert_tm(e, tm)


def _convert_tm(e: Element, tm: CMatrix) -> CMatrix:
    """Convert from xsd-`BaseObjectMl: Matrix` to ecore-`tm: CMatrix`."""
    tm.v0x = get_float(e.find('v0/x')) * MM2M
    tm.v0y = get_float(e.find('v0/y')) * MM2M
    tm.v0z = get_float(e.find('v0/z')) * MM2M
    tm.v1x = get_float(e.find('v1/x'))
    tm.v1y = get_float(e.find('v1/y'))
    tm.v1z = get_float(e.find('v1/z'))
    tm.v2x = get_float(e.find('v2/x'))
    tm.v2y = get_float(e.find('v2/y'))
    tm.v2z = get_float(e.find('v2/z'))
    tm.v3x = get_float(e.find('v3/x'))
    tm.v3y = get_float(e.find('v3/y'))
    tm.v3z = get_float(e.find('v3/z'))
    return tm


def convert_tolerance2d(e: Element) -> Tuple[float, float]:
    """Extract `dX,dY`from xsd/xml."""
    dx = get_float(e.find('Properties/Tolerance/dX')) * MM2M
    dy = get_float(e.find('Properties/Tolerance/dY')) * MM2M
    return dx, dy


class WorkplanLoader():
    """Supports TaskPlan-2.1 only."""

    def __init__(self, rh: ResourceHandler) -> None:
        self.rh: ResourceHandler = rh

    def load_workplan_from_taskplan(self, tp_file) -> CWorkPlan:

        tp: Element = ElementTree.parse(tp_file).getroot()

        wp: CWorkPlan = self.rh.e_class(CWorkPlan)()

        id_root_pattern_id: int = 0
        for nx in tp.findall('Drilling/NodeTree/Node'):
            self._recurse_node(nx, None, wp, id_root_pattern_id)
            id_root_pattern_id += 1
        [wp.nodeTree.append(n) for n in wp.nodes if (n.parent is None)]

        for mx in tp.findall('OnSiteMarkerList1D/OnSiteMarker'):
            wp.markers.append(self._conv_marker(mx))

        for vjx in tp.findall('Moving/MoveJobList1D/MoveJob'):
            wp.jobs.append(self._conv_movejob(vjx))

        for mjx in tp.findall('Measuring/MeasureJobList1D/MeasureJob'):
            wp.jobs.append(self._conv_measurejob(mjx, wp.markers))

        for djx in tp.findall('Drilling/DrillJobList1D/DrillJob'):
            wp.jobs.append(self._conv_drilljob(djx, wp.markers, wp.drillMasks))

        for jrx in tp.findall('JobRefList1D/*'):
            if jrx.tag == 'DestMoveJobRef':
                wp.destMoveJob = self._conv_jobref(jrx, wp.jobs)
            else:
                wp.jobSchedule.append(self._conv_jobref(jrx, wp.jobs))

        # TODO GLobal_Data, Drilling-Cookie, Measuring-Cookie
        # FIXME DestMoveJob
        return wp

    def _recurse_node(self, nx: Element, parent: CNode, wp: CWorkPlan, counter_node: int):
        n: CNode = self.rh.e_class(CNode)()
        if parent is None:
            n.id = f'n{Q}{counter_node}'
        else:
            n.parent, n.id = parent, f'{parent.id}.{counter_node}'
            parent.children.add(n)
        n.name = get_attrib('name', nx)
        n.series = get_attrib('series', nx)
        n.desc = get_attrib('desc', nx)
        n.tm = conv_tml(nx, self.rh.e_class(CMatrix)())
        if fx := nx.find('FastenerTemplate'):
            n.fastener = self._conv_fastener(fx, n, wp)
        incr_child_cnt: int = 0
        for child_nx in nx.findall('Node'):
            self._recurse_node(child_nx, n, wp, incr_child_cnt)
            incr_child_cnt += 1
        wp.nodes.append(n)

    def _conv_fastener(self, fx: Element, n: CNode, wp: CWorkPlan) -> CFastener:
        f: CFastener = self.rh.e_class(CFastener)()
        f.parent, f.name, f.id = n, get_attrib('name', fx), f'{n.id}{Q}f'
        f.dx, f.dy = convert_tolerance2d(fx)
        f.guid = get_attrib('guid', fx)
        cnt_dm: int = 0
        for dmx in fx.findall('DrillMask'):
            dm: CDrillMask = self.rh.e_class(CDrillMask)()
            dm.parent, dm.name, dm.id = f, get_attrib('name', dmx), f'{f.id}{Q}dm{Q}{cnt_dm}'
            dm.dx, dm.dy = convert_tolerance2d(dmx)
            dm.tm = conv_tml(dmx, self.rh.e_class(CMatrix)())
            cnt_dh: int = 0
            for dhx in dmx.findall('DrillHole'):
                dh: CDrillHole = self.rh.e_class(CDrillHole)()
                dh.parent, dh.name, dh.id = dm, get_attrib('name', dhx), f'{dm.id}{Q}dh{Q}{cnt_dh}'
                dh.dx, dh.dy = convert_tolerance2d(dhx)
                dh.tm = conv_tml(dhx, self.rh.e_class(CMatrix)())
                dh.depth = get_float(dhx.find('Properties/DrillDepth')) * MM2M
                dh.diameter = get_float(dhx.find('Properties/DrillDiameter')) * MM2M
                dh.state = dh.state.eEnum.from_string('UNDRILLED')  # DRILL_HOLE_STATE
                dm.drillHoles.append(dh)
                wp.drillHoles.append(dh)
                cnt_dh += 1
            f.drillMasks.append(dm)
            wp.drillMasks.append(dm)
            cnt_dm += 1
        wp.fasteners.append(f)
        return f

    def _conv_marker(self, mx: Element) -> CMarker:
        m: CMarker = self.rh.e_class(CMarker)()
        m.name = get_attrib('name', mx)
        m.id = f'm{Q}{m.name}'
        m.parent = get_attrib('parent', mx)
        m.code = get_attrib('code', mx)
        m.time = get_attrib('time', mx)
        m.state = m.state.eEnum.from_string(get_attrib('state', mx, 'UNDEFINED'))
        m.tm = conv_tmg(mx, self.rh.e_class(CMatrix)())
        return m

    def _conv_movejob(self, vjx: Element) -> CMoveJob:
        vj = self._fill_job(self.rh.e_class(CMoveJob)(), vjx, [])
        vj.id = f'vj{Q}{vj.id}'
        return vj

    def _conv_measurejob(self, mjx: Element, markers) -> CMeasureJob:
        mj = self._fill_job(self.rh.e_class(CMeasureJob)(), mjx, markers)
        mj.id = f'mj{Q}{mj.id}'
        return mj

    def _fill_job(self, j: CJob, jx: Element, markers: List[CMarker]):
        j.name = get_attrib('name', jx)
        j.id = j.name
        j.tm = conv_tmg(jx, self.rh.e_class(CMatrix)())
        for marker_ref in jx.findall('JobProperties/OnSiteMarkerRefList1D/OnSiteMarkerRef'):
            old = len(j.marker)
            [j.marker.append(m) for m in markers if (m.name == marker_ref.text)]
            if len(j.marker) - old < 1:
                raise Exception(
                    f"Job '{j.name}' refs not existing Marker '{marker_ref.text}'")
        return j

    def _conv_drilljob(self, djx: Element, markers, drillmasks: List[CDrillMask]) -> CDrillJob:
        dj: CDrillJob = self._fill_job(self.rh.e_class(CDrillJob)(), djx, markers)
        dj.id = f'dj{Q}{dj.id}'
        for drillmask_ref in djx.findall('JobProperties/DrillMaskRefList1D/DrillMaskRef'):
            found = False
            for drillmask in drillmasks:
                if drillmask.name == drillmask_ref.text:
                    found = True
                    dj.drillMasks.append(drillmask)
            if not found:
                raise Exception(
                    f"DrillJob '{dj.name}' refers not existing DrillMask '{drillmask_ref.text}'")
        return dj

    def _conv_jobref(self, jrx: Element, jobs: List[CJob]) -> CJob:
        for j in jobs:
            if j.name == jrx.text:
                return j
        raise Exception(f"JobRef refers not existing Job '{jrx.text}'")
