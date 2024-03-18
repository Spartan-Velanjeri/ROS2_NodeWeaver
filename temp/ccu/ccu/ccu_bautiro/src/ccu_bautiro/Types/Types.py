"""Definition of meta model 'Types'."""
from functools import partial
import pyecore.ecore as Ecore
from pyecore.ecore import *


name = 'Types'
nsURI = 'http://www.bosch.com/bautiro/CTypes'
nsPrefix = 'CTY'

eClass = EPackage(name=name, nsURI=nsURI, nsPrefix=nsPrefix)

eClassifiers = {}
getEClassifier = partial(Ecore.getEClassifier, searchspace=eClassifiers)
MATERIAL_TYPE = EEnum('MATERIAL_TYPE', literals=['UNDEFINED', 'OTHER', 'CONCRETE'])

TP_XSD_VERSION = EEnum('TP_XSD_VERSION', literals=['UNDEFINED', 'TaskPlan_2_0', 'TaskPlan_2_1'])

IMPORT_SOURCE = EEnum('IMPORT_SOURCE', literals=['UNDEFINED', 'USB', 'HCU', 'MISSION'])

MARKER_STATE = EEnum('MARKER_STATE', literals=['UNDEFINED', 'planned', 'measured'])

DRILL_HOLE_STATE = EEnum('DRILL_HOLE_STATE', literals=[
                         'UNDEFINED', 'UNDRILLED', 'SCHEDULED', 'FINISHED', 'IGNORE', 'FAILED'])


class CMatrix(EObject, metaclass=MetaEClass):

    v0x = EAttribute(eType=EDouble, unique=True, derived=False, changeable=True)
    v0y = EAttribute(eType=EDouble, unique=True, derived=False, changeable=True)
    v0z = EAttribute(eType=EDouble, unique=True, derived=False, changeable=True)
    v1x = EAttribute(eType=EDouble, unique=True, derived=False, changeable=True)
    v1y = EAttribute(eType=EDouble, unique=True, derived=False, changeable=True)
    v1z = EAttribute(eType=EDouble, unique=True, derived=False, changeable=True)
    v2x = EAttribute(eType=EDouble, unique=True, derived=False, changeable=True)
    v2y = EAttribute(eType=EDouble, unique=True, derived=False, changeable=True)
    v2z = EAttribute(eType=EDouble, unique=True, derived=False, changeable=True)
    v3x = EAttribute(eType=EDouble, unique=True, derived=False, changeable=True)
    v3y = EAttribute(eType=EDouble, unique=True, derived=False, changeable=True)
    v3z = EAttribute(eType=EDouble, unique=True, derived=False, changeable=True)

    def __init__(self, *, v0x=None, v0y=None, v0z=None, v1x=None, v1y=None, v1z=None, v2x=None, v2y=None, v2z=None, v3x=None, v3y=None, v3z=None):
        # if kwargs:
        #    raise AttributeError('unexpected arguments: {}'.format(kwargs))

        super().__init__()

        if v0x is not None:
            self.v0x = v0x

        if v0y is not None:
            self.v0y = v0y

        if v0z is not None:
            self.v0z = v0z

        if v1x is not None:
            self.v1x = v1x

        if v1y is not None:
            self.v1y = v1y

        if v1z is not None:
            self.v1z = v1z

        if v2x is not None:
            self.v2x = v2x

        if v2y is not None:
            self.v2y = v2y

        if v2z is not None:
            self.v2z = v2z

        if v3x is not None:
            self.v3x = v3x

        if v3y is not None:
            self.v3y = v3y

        if v3z is not None:
            self.v3z = v3z


class KeyValuePair(EObject, metaclass=MetaEClass):

    key = EAttribute(eType=EString, unique=True, derived=False, changeable=True)
    value = EAttribute(eType=EString, unique=True, derived=False, changeable=True)

    def __init__(self, *, key=None, value=None):
        # if kwargs:
        #    raise AttributeError('unexpected arguments: {}'.format(kwargs))

        super().__init__()

        if key is not None:
            self.key = key

        if value is not None:
            self.value = value


@abstract
class ICreationDateTime(EObject, metaclass=MetaEClass):

    dateTimeCreation = EAttribute(eType=EString, unique=True, derived=False, changeable=True)

    def __init__(self, *, dateTimeCreation=None):
        # if kwargs:
        #    raise AttributeError('unexpected arguments: {}'.format(kwargs))

        super().__init__()

        if dateTimeCreation is not None:
            self.dateTimeCreation = dateTimeCreation


@abstract
class IID(EObject, metaclass=MetaEClass):

    id = EAttribute(eType=EString, unique=True, derived=False, changeable=True, iD=True)

    def __init__(self, *, id=None):
        # if kwargs:
        #    raise AttributeError('unexpected arguments: {}'.format(kwargs))

        super().__init__()

        if id is not None:
            self.id = id


@abstract
class IINDEX(EObject, metaclass=MetaEClass):

    index = EAttribute(eType=ELong, unique=True, derived=False, changeable=True)

    def __init__(self, *, index=None):
        # if kwargs:
        #    raise AttributeError('unexpected arguments: {}'.format(kwargs))

        super().__init__()

        if index is not None:
            self.index = index


@abstract
class INAME(EObject, metaclass=MetaEClass):

    name = EAttribute(eType=EString, unique=True, derived=False, changeable=True)

    def __init__(self, *, name=None):
        # if kwargs:
        #    raise AttributeError('unexpected arguments: {}'.format(kwargs))

        super().__init__()

        if name is not None:
            self.name = name


@abstract
class ITM(EObject, metaclass=MetaEClass):

    tm = EReference(ordered=True, unique=True, containment=True, derived=False)

    def __init__(self, *, tm=None):
        # if kwargs:
        #    raise AttributeError('unexpected arguments: {}'.format(kwargs))

        super().__init__()

        if tm is not None:
            self.tm = tm


@abstract
class ITolerance2D(EObject, metaclass=MetaEClass):

    dx = EAttribute(eType=EDouble, unique=True, derived=False, changeable=True, default_value=0.0)
    dy = EAttribute(eType=EDouble, unique=True, derived=False, changeable=True, default_value=0.0)

    def __init__(self, *, dx=None, dy=None):
        # if kwargs:
        #    raise AttributeError('unexpected arguments: {}'.format(kwargs))

        super().__init__()

        if dx is not None:
            self.dx = dx

        if dy is not None:
            self.dy = dy


class CPose(EObject, metaclass=MetaEClass):

    vx = EAttribute(eType=EDouble, unique=True, derived=False, changeable=True)
    vy = EAttribute(eType=EDouble, unique=True, derived=False, changeable=True)
    vz = EAttribute(eType=EDouble, unique=True, derived=False, changeable=True)
    qx = EAttribute(eType=EDouble, unique=True, derived=False, changeable=True)
    qy = EAttribute(eType=EDouble, unique=True, derived=False, changeable=True)
    qz = EAttribute(eType=EDouble, unique=True, derived=False, changeable=True)
    qw = EAttribute(eType=EDouble, unique=True, derived=False, changeable=True)

    def __init__(self, *, vx=None, vy=None, vz=None, qx=None, qy=None, qz=None, qw=None):
        # if kwargs:
        #    raise AttributeError('unexpected arguments: {}'.format(kwargs))

        super().__init__()

        if vx is not None:
            self.vx = vx

        if vy is not None:
            self.vy = vy

        if vz is not None:
            self.vz = vz

        if qx is not None:
            self.qx = qx

        if qy is not None:
            self.qy = qy

        if qz is not None:
            self.qz = qz

        if qw is not None:
            self.qw = qw


class CImage(EObject, metaclass=MetaEClass):

    drawingType = EAttribute(eType=EString, unique=True, derived=False, changeable=True)
    faceType = EAttribute(eType=EString, unique=True, derived=False, changeable=True)
    filename = EAttribute(eType=EString, unique=True, derived=False, changeable=True)

    def __init__(self, *, drawingType=None, faceType=None, filename=None):
        # if kwargs:
        #    raise AttributeError('unexpected arguments: {}'.format(kwargs))

        super().__init__()

        if drawingType is not None:
            self.drawingType = drawingType

        if faceType is not None:
            self.faceType = faceType

        if filename is not None:
            self.filename = filename


class Cookie(IID, INAME):

    property = EReference(ordered=True, unique=True, containment=True, derived=False, upper=-1)
    cookie = EReference(ordered=True, unique=True, containment=False, derived=False, upper=-1)
    parent = EReference(ordered=True, unique=True, containment=False, derived=False)

    def __init__(self, *, property=None, cookie=None, parent=None, **kwargs):

        super().__init__(**kwargs)

        if property:
            self.property.extend(property)

        if cookie:
            self.cookie.extend(cookie)

        if parent is not None:
            self.parent = parent


class CMarker(IID, INAME, ITM):

    code = EAttribute(eType=EString, unique=True, derived=False, changeable=True)
    parent = EAttribute(eType=EString, unique=True, derived=False, changeable=True)
    time = EAttribute(eType=EString, unique=True, derived=False, changeable=True)
    state = EAttribute(eType=MARKER_STATE, unique=True, derived=False, changeable=True)

    def __init__(self, *, code=None, parent=None, time=None, state=None, **kwargs):

        super().__init__(**kwargs)

        if code is not None:
            self.code = code

        if parent is not None:
            self.parent = parent

        if time is not None:
            self.time = time

        if state is not None:
            self.state = state
