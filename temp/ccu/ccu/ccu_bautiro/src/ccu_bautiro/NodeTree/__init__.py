
from .NodeTree import getEClassifier, eClassifiers
from .NodeTree import name, nsURI, nsPrefix, eClass
from .NodeTree import CNode, CFastener, CDrillMask, CDrillHole


from . import NodeTree
from .. import ccu_bautiro


__all__ = ['CNode', 'CFastener', 'CDrillMask', 'CDrillHole']

eSubpackages = []
eSuperPackage = ccu_bautiro
NodeTree.eSubpackages = eSubpackages
NodeTree.eSuperPackage = eSuperPackage


otherClassifiers = []

for classif in otherClassifiers:
    eClassifiers[classif.name] = classif
    classif.ePackage = eClass

for classif in eClassifiers.values():
    eClass.eClassifiers.append(classif.eClass)

for subpack in eSubpackages:
    eClass.eSubpackages.append(subpack.eClass)
