
from .ccu_bautiro import getEClassifier, eClassifiers
from .ccu_bautiro import name, nsURI, nsPrefix, eClass
from .ccu_bautiro import CWorkPlan, CMission, CProcessData

from ccu_bautiro.Types import Cookie, CMarker
from ccu_bautiro.NodeTree import CFastener, CNode, CDrillHole, CDrillMask
from ccu_bautiro.Jobs import CMoveJob, CJob
from ccu_bautiro.ProcessData import CNavigationPD, CDrillHolePD

from .MetaData import CTaskPlanMD, CRoomPlanMD
from .NodeTree import CFastener, CNode, CDrillHole, CDrillMask
from .ProcessData import CNavigationPD, CDrillHolePD
from .Jobs import CMoveJob, CJob, CDrillJob
from .Types import CMarker, CImage, CPose, KeyValuePair, Cookie, CMatrix, ITM
from . import ccu_bautiro
from . import MetaData

from . import NodeTree

from . import ProcessData

from . import Jobs

from . import Types


__all__ = ['CWorkPlan', 'CMission', 'CProcessData']

eSubpackages = [MetaData, NodeTree, ProcessData, Jobs, Types]
eSuperPackage = None
ccu_bautiro.eSubpackages = eSubpackages
ccu_bautiro.eSuperPackage = eSuperPackage

CWorkPlan.nodeTree.eType = CNode
CWorkPlan.fasteners.eType = CFastener
CWorkPlan.drillMasks.eType = CDrillMask
CWorkPlan.drillHoles.eType = CDrillHole
CWorkPlan.markers.eType = CMarker
CWorkPlan.jobs.eType = CJob
CWorkPlan.jobSchedule.eType = CJob
CWorkPlan.destMoveJob.eType = CMoveJob
CWorkPlan.drillingCookie.eType = Cookie
CWorkPlan.measureCookie.eType = Cookie
CWorkPlan.nodes.eType = CNode
CMission.jobs.eType = CJob
CMission.workPlan.eType = CWorkPlan
CProcessData.drillHolePD.eType = CDrillHolePD
CProcessData.navigationPD.eType = CNavigationPD
CTaskPlanMD.globalSettings_MarkerPlan_Images.eType = CImage
CTaskPlanMD.roomPlanMD.eType = CRoomPlanMD
CDrillHolePD.absPosePKSFinish.eType = CPose
CDrillHolePD.absPosePKSStart.eType = CPose
CDrillHolePD.drillHole.eType = CDrillHole
CNavigationPD.job.eType = CJob
CJob.marker.eType = CMarker
CDrillJob.drillMasks.eType = CDrillMask
Cookie.property.eType = KeyValuePair
ITM.tm.eType = CMatrix
CNode.fastener.eType = CFastener
CNode.children.eType = CNode
CNode.parent.eType = CNode
CNode.parent.eOpposite = CNode.children
CFastener.drillMasks.eType = CDrillMask
CFastener.parent.eType = CNode
CFastener.parent.eOpposite = CNode.fastener
CDrillMask.drillHoles.eType = CDrillHole
CDrillMask.parent.eType = CFastener
CDrillMask.parent.eOpposite = CFastener.drillMasks
CDrillHole.parent.eType = CDrillMask
CDrillHole.parent.eOpposite = CDrillMask.drillHoles
Cookie.cookie.eType = Cookie
Cookie.parent.eType = Cookie
Cookie.parent.eOpposite = Cookie.cookie

otherClassifiers = []

for classif in otherClassifiers:
    eClassifiers[classif.name] = classif
    classif.ePackage = eClass

for classif in eClassifiers.values():
    eClass.eClassifiers.append(classif.eClass)

for subpack in eSubpackages:
    eClass.eSubpackages.append(subpack.eClass)
