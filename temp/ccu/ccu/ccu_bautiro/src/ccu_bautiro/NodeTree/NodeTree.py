"""Definition of meta model 'NodeTree'."""
from functools import partial
import pyecore.ecore as Ecore
from pyecore.ecore import *
from ccu_bautiro.Types import IID, ITolerance2D, DRILL_HOLE_STATE, INAME, ITM


name = 'NodeTree'
nsURI = 'http://www.bosch.com/bautiro/CNodeTree'
nsPrefix = 'CNT'

eClass = EPackage(name=name, nsURI=nsURI, nsPrefix=nsPrefix)

eClassifiers = {}
getEClassifier = partial(Ecore.getEClassifier, searchspace=eClassifiers)


class CNode(IID, INAME, ITM):

    desc = EAttribute(eType=EString, unique=True, derived=False, changeable=True)
    series = EAttribute(eType=EString, unique=True, derived=False, changeable=True)
    fastener = EReference(ordered=True, unique=True, containment=False, derived=False)
    children = EReference(ordered=True, unique=True, containment=False, derived=False, upper=-1)
    parent = EReference(ordered=True, unique=True, containment=False, derived=False)

    def __init__(self, *, desc=None, series=None, fastener=None, children=None, parent=None, **kwargs):

        super().__init__(**kwargs)

        if desc is not None:
            self.desc = desc

        if series is not None:
            self.series = series

        if fastener is not None:
            self.fastener = fastener

        if children:
            self.children.extend(children)

        if parent is not None:
            self.parent = parent


class CFastener(IID, INAME, ITolerance2D):

    guid = EAttribute(eType=EString, unique=True, derived=False, changeable=True)
    drillMasks = EReference(ordered=True, unique=True, containment=False, derived=False, upper=-1)
    parent = EReference(ordered=True, unique=True, containment=False, derived=False)

    def __init__(self, *, guid=None, drillMasks=None, parent=None, **kwargs):

        super().__init__(**kwargs)

        if guid is not None:
            self.guid = guid

        if drillMasks:
            self.drillMasks.extend(drillMasks)

        if parent is not None:
            self.parent = parent


class CDrillMask(IID, INAME, ITM, ITolerance2D):

    drillHoles = EReference(ordered=True, unique=True, containment=False, derived=False, upper=-1)
    parent = EReference(ordered=True, unique=True, containment=False, derived=False)

    def __init__(self, *, drillHoles=None, parent=None, **kwargs):

        super().__init__(**kwargs)

        if drillHoles:
            self.drillHoles.extend(drillHoles)

        if parent is not None:
            self.parent = parent


class CDrillHole(IID, INAME, ITM, ITolerance2D):

    depth = EAttribute(eType=EDouble, unique=True, derived=False, changeable=True)
    diameter = EAttribute(eType=EDouble, unique=True, derived=False, changeable=True)
    state = EAttribute(eType=DRILL_HOLE_STATE, unique=True, derived=False, changeable=True)
    parent = EReference(ordered=True, unique=True, containment=False, derived=False)

    def __init__(self, *, parent=None, depth=None, diameter=None, state=None, **kwargs):

        super().__init__(**kwargs)

        if depth is not None:
            self.depth = depth

        if diameter is not None:
            self.diameter = diameter

        if state is not None:
            self.state = state

        if parent is not None:
            self.parent = parent
