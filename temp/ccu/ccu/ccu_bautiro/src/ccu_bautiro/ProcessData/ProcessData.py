"""Definition of meta model 'ProcessData'."""
from functools import partial
import pyecore.ecore as Ecore
from pyecore.ecore import *
from ccu_bautiro.Types import IID


name = 'ProcessData'
nsURI = 'http://www.bosch.com/bautiro/CProcessData'
nsPrefix = 'CPD'

eClass = EPackage(name=name, nsURI=nsURI, nsPrefix=nsPrefix)

eClassifiers = {}
getEClassifier = partial(Ecore.getEClassifier, searchspace=eClassifiers)


class CDrillHolePD(IID):

    absPosePKSFinish = EReference(ordered=True, unique=True, containment=False, derived=False)
    absPosePKSStart = EReference(ordered=True, unique=True, containment=False, derived=False)
    drillHole = EReference(ordered=True, unique=True, containment=False, derived=False)

    def __init__(self, *, absPosePKSFinish=None, absPosePKSStart=None, drillHole=None, **kwargs):

        super().__init__(**kwargs)

        if absPosePKSFinish is not None:
            self.absPosePKSFinish = absPosePKSFinish

        if absPosePKSStart is not None:
            self.absPosePKSStart = absPosePKSStart

        if drillHole is not None:
            self.drillHole = drillHole


class CNavigationPD(IID):

    nav_goal_reached = EAttribute(eType=EBoolean, unique=True, derived=False, changeable=True)
    job = EReference(ordered=True, unique=True, containment=False, derived=False)

    def __init__(self, *, nav_goal_reached=None, job=None, **kwargs):

        super().__init__(**kwargs)

        if nav_goal_reached is not None:
            self.nav_goal_reached = nav_goal_reached

        if job is not None:
            self.job = job
