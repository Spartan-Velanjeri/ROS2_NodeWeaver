
from .MetaData import getEClassifier, eClassifiers
from .MetaData import name, nsURI, nsPrefix, eClass
from .MetaData import CMetaData, CRoomPlanMD, CTaskPlanMD, CWorkPlanMD, CMissionMD


from . import MetaData
from .. import ccu_bautiro


__all__ = ['CMetaData', 'CRoomPlanMD', 'CTaskPlanMD', 'CWorkPlanMD', 'CMissionMD']

eSubpackages = []
eSuperPackage = ccu_bautiro
MetaData.eSubpackages = eSubpackages
MetaData.eSuperPackage = eSuperPackage


otherClassifiers = []

for classif in otherClassifiers:
    eClassifiers[classif.name] = classif
    classif.ePackage = eClass

for classif in eClassifiers.values():
    eClass.eClassifiers.append(classif.eClass)

for subpack in eSubpackages:
    eClass.eSubpackages.append(subpack.eClass)
