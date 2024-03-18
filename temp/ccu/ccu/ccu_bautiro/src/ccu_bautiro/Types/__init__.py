
from .Types import getEClassifier, eClassifiers
from .Types import name, nsURI, nsPrefix, eClass
from .Types import MATERIAL_TYPE, TP_XSD_VERSION, IMPORT_SOURCE, MARKER_STATE, DRILL_HOLE_STATE, CMarker, CMatrix, Cookie, KeyValuePair, ICreationDateTime, IID, IINDEX, INAME, ITM, ITolerance2D, CPose, CImage


from . import Types
from .. import ccu_bautiro


__all__ = ['MATERIAL_TYPE', 'TP_XSD_VERSION', 'IMPORT_SOURCE', 'MARKER_STATE', 'DRILL_HOLE_STATE', 'CMarker', 'CMatrix',
           'Cookie', 'KeyValuePair', 'ICreationDateTime', 'IID', 'IINDEX', 'INAME', 'ITM', 'ITolerance2D', 'CPose', 'CImage']

eSubpackages = []
eSuperPackage = ccu_bautiro
Types.eSubpackages = eSubpackages
Types.eSuperPackage = eSuperPackage


otherClassifiers = [MATERIAL_TYPE, TP_XSD_VERSION, IMPORT_SOURCE, MARKER_STATE, DRILL_HOLE_STATE]

for classif in otherClassifiers:
    eClassifiers[classif.name] = classif
    classif.ePackage = eClass

for classif in eClassifiers.values():
    eClass.eClassifiers.append(classif.eClass)

for subpack in eSubpackages:
    eClass.eSubpackages.append(subpack.eClass)
