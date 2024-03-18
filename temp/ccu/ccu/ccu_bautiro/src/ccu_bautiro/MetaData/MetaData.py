"""Definition of meta model 'MetaData'."""
from functools import partial
import pyecore.ecore as Ecore
from pyecore.ecore import *
from ccu_bautiro.Types import ICreationDateTime, IID, IMPORT_SOURCE, IINDEX, TP_XSD_VERSION, INAME


name = 'MetaData'
nsURI = 'http://www.bosch.com/bautiro/CMetaData'
nsPrefix = 'CMD'

eClass = EPackage(name=name, nsURI=nsURI, nsPrefix=nsPrefix)

eClassifiers = {}
getEClassifier = partial(Ecore.getEClassifier, searchspace=eClassifiers)


@abstract
class CMetaData(IID, IINDEX, INAME, ICreationDateTime):

    relPath = EAttribute(eType=EString, unique=True, derived=False, changeable=True)
    originImportSource = EAttribute(eType=IMPORT_SOURCE, unique=True,
                                    derived=False, changeable=True)
    originFileName = EAttribute(eType=EString, unique=True, derived=False, changeable=True)
    originFilePath = EAttribute(eType=EString, unique=True, derived=False, changeable=True)

    def __init__(self, *, relPath=None, originImportSource=None, originFileName=None, originFilePath=None, **kwargs):

        super().__init__(**kwargs)

        if relPath is not None:
            self.relPath = relPath

        if originImportSource is not None:
            self.originImportSource = originImportSource

        if originFileName is not None:
            self.originFileName = originFileName

        if originFilePath is not None:
            self.originFilePath = originFilePath


class CRoomPlanMD(CMetaData):

    calculated_MD5_32bitHex = EAttribute(eType=EString, unique=True, derived=False, changeable=True)

    def __init__(self, *, calculated_MD5_32bitHex=None, **kwargs):

        super().__init__(**kwargs)

        if calculated_MD5_32bitHex is not None:
            self.calculated_MD5_32bitHex = calculated_MD5_32bitHex


class CTaskPlanMD(CMetaData):

    globalSettings_RoomPlan_Filename = EAttribute(
        eType=EString, unique=True, derived=False, changeable=True)
    globalSettings_RoomPlan_MD5_32bitHex = EAttribute(
        eType=EString, unique=True, derived=False, changeable=True)
    globalSettings_Schedule_desiredStartTime = EAttribute(
        eType=EString, unique=True, derived=False, changeable=True)
    taskPlanXsdVersion = EAttribute(eType=TP_XSD_VERSION, unique=True,
                                    derived=False, changeable=True, default_value=TP_XSD_VERSION.UNDEFINED)
    roomPlanIndex = EAttribute(eType=ELong, unique=True, derived=False, changeable=True)
    taskname = EAttribute(eType=EString, unique=True, derived=False, changeable=True)
    version = EAttribute(eType=EString, unique=True, derived=False, changeable=True)
    tool = EAttribute(eType=EString, unique=True, derived=False,
                      changeable=True, default_value='BAUTIRO')
    globalSettings_MarkerPlan_Images = EReference(
        ordered=True, unique=True, containment=True, derived=False, upper=-1)
    roomPlanMD = EReference(ordered=True, unique=True, containment=False, derived=False)

    def __init__(self, *, globalSettings_RoomPlan_Filename=None, globalSettings_RoomPlan_MD5_32bitHex=None, globalSettings_MarkerPlan_Images=None, globalSettings_Schedule_desiredStartTime=None, taskPlanXsdVersion=None, roomPlanIndex=None, roomPlanMD=None, taskname=None, version=None, tool=None, **kwargs):

        super().__init__(**kwargs)

        if globalSettings_RoomPlan_Filename is not None:
            self.globalSettings_RoomPlan_Filename = globalSettings_RoomPlan_Filename

        if globalSettings_RoomPlan_MD5_32bitHex is not None:
            self.globalSettings_RoomPlan_MD5_32bitHex = globalSettings_RoomPlan_MD5_32bitHex

        if globalSettings_Schedule_desiredStartTime is not None:
            self.globalSettings_Schedule_desiredStartTime = globalSettings_Schedule_desiredStartTime

        if taskPlanXsdVersion is not None:
            self.taskPlanXsdVersion = taskPlanXsdVersion

        if roomPlanIndex is not None:
            self.roomPlanIndex = roomPlanIndex

        if taskname is not None:
            self.taskname = taskname

        if version is not None:
            self.version = version

        if tool is not None:
            self.tool = tool

        if globalSettings_MarkerPlan_Images:
            self.globalSettings_MarkerPlan_Images.extend(globalSettings_MarkerPlan_Images)

        if roomPlanMD is not None:
            self.roomPlanMD = roomPlanMD


class CWorkPlanMD(CMetaData):

    taskPlanIndex = EAttribute(eType=ELong, unique=True, derived=False, changeable=True)
    roomPlanIndex = EAttribute(eType=ELong, unique=True, derived=False, changeable=True)

    def __init__(self, *, taskPlanIndex=None, roomPlanIndex=None, **kwargs):

        super().__init__(**kwargs)

        if taskPlanIndex is not None:
            self.taskPlanIndex = taskPlanIndex

        if roomPlanIndex is not None:
            self.roomPlanIndex = roomPlanIndex


class CMissionMD(CMetaData):

    workPlanIndex = EAttribute(eType=ELong, unique=True, derived=False, changeable=True)
    predecessorIDs = EAttribute(eType=ELong, unique=True, derived=False, changeable=True, upper=-1)

    def __init__(self, *, workPlanIndex=None, predecessorIDs=None, **kwargs):

        super().__init__(**kwargs)

        if workPlanIndex is not None:
            self.workPlanIndex = workPlanIndex

        if predecessorIDs:
            self.predecessorIDs.extend(predecessorIDs)
