"""Definition of meta model 'ccu_bautiro'."""
from functools import partial
import pyecore.ecore as Ecore
from pyecore.ecore import *
from ccu_bautiro.Types import IINDEX, ICreationDateTime, IID


name = 'ccu_bautiro'
nsURI = 'http://www.bosch.com/ccu_bautiro'
nsPrefix = 'CB'

eClass = EPackage(name=name, nsURI=nsURI, nsPrefix=nsPrefix)

eClassifiers = {}
getEClassifier = partial(Ecore.getEClassifier, searchspace=eClassifiers)


class CWorkPlan(IID, IINDEX):

    nodeTree = EReference(ordered=True, unique=True, containment=False, derived=False, upper=-1)
    fasteners = EReference(ordered=True, unique=True, containment=True, derived=False, upper=-1)
    drillMasks = EReference(ordered=True, unique=True, containment=True, derived=False, upper=-1)
    drillHoles = EReference(ordered=True, unique=True, containment=True, derived=False, upper=-1)
    markers = EReference(ordered=True, unique=True, containment=True, derived=False, upper=-1)
    jobs = EReference(ordered=True, unique=True, containment=True, derived=False, upper=-1)
    jobSchedule = EReference(ordered=True, unique=True, containment=False, derived=False, upper=-1)
    destMoveJob = EReference(ordered=True, unique=True, containment=False, derived=False)
    drillingCookie = EReference(ordered=True, unique=True,
                                containment=True, derived=False, upper=-1)
    measureCookie = EReference(ordered=True, unique=True, containment=True, derived=False, upper=-1)
    nodes = EReference(ordered=True, unique=True, containment=True, derived=False, upper=-1)

    def __init__(self, *, nodeTree=None, fasteners=None, drillMasks=None, drillHoles=None, markers=None, jobs=None, jobSchedule=None, destMoveJob=None, drillingCookie=None, measureCookie=None, nodes=None, **kwargs):

        super().__init__(**kwargs)

        if nodeTree:
            self.nodeTree.extend(nodeTree)

        if fasteners:
            self.fasteners.extend(fasteners)

        if drillMasks:
            self.drillMasks.extend(drillMasks)

        if drillHoles:
            self.drillHoles.extend(drillHoles)

        if markers:
            self.markers.extend(markers)

        if jobs:
            self.jobs.extend(jobs)

        if jobSchedule:
            self.jobSchedule.extend(jobSchedule)

        if destMoveJob is not None:
            self.destMoveJob = destMoveJob

        if drillingCookie:
            self.drillingCookie.extend(drillingCookie)

        if measureCookie:
            self.measureCookie.extend(measureCookie)

        if nodes:
            self.nodes.extend(nodes)


class CMission(IID, IINDEX):

    jobs = EReference(ordered=True, unique=True, containment=False, derived=False, upper=-1)
    workPlan = EReference(ordered=True, unique=True, containment=False, derived=False)

    def __init__(self, *, jobs=None, workPlan=None, **kwargs):

        super().__init__(**kwargs)

        if jobs:
            self.jobs.extend(jobs)

        if workPlan is not None:
            self.workPlan = workPlan


class CProcessData(IID, IINDEX, ICreationDateTime):

    workPlanIndex = EAttribute(eType=ELong, unique=True, derived=False, changeable=True)
    missionIndex = EAttribute(eType=ELong, unique=True, derived=False, changeable=True)
    drillHolePD = EReference(ordered=True, unique=True, containment=True, derived=False, upper=-1)
    navigationPD = EReference(ordered=True, unique=True, containment=True, derived=False, upper=-1)

    def __init__(self, *, workPlanIndex=None, missionIndex=None, drillHolePD=None, navigationPD=None, **kwargs):

        super().__init__(**kwargs)

        if workPlanIndex is not None:
            self.workPlanIndex = workPlanIndex

        if missionIndex is not None:
            self.missionIndex = missionIndex

        if drillHolePD:
            self.drillHolePD.extend(drillHolePD)

        if navigationPD:
            self.navigationPD.extend(navigationPD)
