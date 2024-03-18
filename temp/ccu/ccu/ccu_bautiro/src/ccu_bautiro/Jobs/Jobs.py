"""Definition of meta model 'Jobs'."""
from functools import partial
import pyecore.ecore as Ecore
from pyecore.ecore import *
from ccu_bautiro.Types import INAME, ITM, MATERIAL_TYPE, IID


name = 'Jobs'
nsURI = 'http://www.bosch.com/bautiro/CJobs'
nsPrefix = 'CJ'

eClass = EPackage(name=name, nsURI=nsURI, nsPrefix=nsPrefix)

eClassifiers = {}
getEClassifier = partial(Ecore.getEClassifier, searchspace=eClassifiers)


@abstract
class CJob(IID, INAME, ITM):

    marker = EReference(ordered=True, unique=True, containment=False, derived=False, upper=-1)

    def __init__(self, *, marker=None, **kwargs):

        super().__init__(**kwargs)

        if marker:
            self.marker.extend(marker)


class CDrillJob(CJob):

    material = EAttribute(eType=MATERIAL_TYPE, unique=True, derived=False, changeable=True)
    drillMasks = EReference(ordered=True, unique=True, containment=False, derived=False, upper=-1)

    def __init__(self, *, drillMasks=None, material=None, **kwargs):

        super().__init__(**kwargs)

        if material is not None:
            self.material = material

        if drillMasks:
            self.drillMasks.extend(drillMasks)


class CMeasureJob(CJob):

    def __init__(self, **kwargs):

        super().__init__(**kwargs)


class CMoveJob(CJob):

    def __init__(self, **kwargs):

        super().__init__(**kwargs)
