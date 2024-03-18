
from .ProcessData import getEClassifier, eClassifiers
from .ProcessData import name, nsURI, nsPrefix, eClass
from .ProcessData import CDrillHolePD, CNavigationPD


from . import ProcessData
from .. import ccu_bautiro


__all__ = ['CDrillHolePD', 'CNavigationPD']

eSubpackages = []
eSuperPackage = ccu_bautiro
ProcessData.eSubpackages = eSubpackages
ProcessData.eSuperPackage = eSuperPackage


otherClassifiers = []

for classif in otherClassifiers:
    eClassifiers[classif.name] = classif
    classif.ePackage = eClass

for classif in eClassifiers.values():
    eClass.eClassifiers.append(classif.eClass)

for subpack in eSubpackages:
    eClass.eSubpackages.append(subpack.eClass)
