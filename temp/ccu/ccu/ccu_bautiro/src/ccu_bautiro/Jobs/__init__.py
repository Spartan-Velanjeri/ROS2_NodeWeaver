
from .Jobs import getEClassifier, eClassifiers
from .Jobs import name, nsURI, nsPrefix, eClass
from .Jobs import CJob, CDrillJob, CMeasureJob, CMoveJob


from . import Jobs
from .. import ccu_bautiro


__all__ = ['CJob', 'CDrillJob', 'CMeasureJob', 'CMoveJob']

eSubpackages = []
eSuperPackage = ccu_bautiro
Jobs.eSubpackages = eSubpackages
Jobs.eSuperPackage = eSuperPackage


otherClassifiers = []

for classif in otherClassifiers:
    eClassifiers[classif.name] = classif
    classif.ePackage = eClass

for classif in eClassifiers.values():
    eClass.eClassifiers.append(classif.eClass)

for subpack in eSubpackages:
    eClass.eSubpackages.append(subpack.eClass)
