# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from os.path import basename, dirname, join
from pathlib import Path

from pkg_resources import resource_filename
from pyecore.ecore import (EClass, EClassifier, EEnum, EObject, EPackage,
                           EProxy, MetaEClass)
from pyecore.resources import URI, ResourceSet
from pyecore.resources.xmi import XMIOptions, XMIResource

CBAUTIRO_ECORE = resource_filename('ccu_bautiro', 'ccu_bautiro.ecore')
XMI_SAVE_OPTIONS = {XMIOptions.SERIALIZE_DEFAULT_VALUES: True,
                    XMIOptions.OPTION_USE_XMI_TYPE: True}


def resolve(e: EObject) -> any:
    """if `e` is a proxy, it returns the proxied element, else `e`."""
    if isinstance(e, EProxy):
        return e.force_resolve()
    return e


def is_same_classname(e_object: EObject, e_class: MetaEClass) -> bool:
    if e_object is None:
        return False
    eo_resolved = resolve(e_object)
    ec_resolved = resolve(e_class)

    class_of_eo: EClass = eo_resolved.eClass
    class_of_ec: EClass = ec_resolved.eClass
    return class_of_eo.name == class_of_ec.name


class ResourceHandler():

    def __init__(self, data_folder_path) -> None:
        self.data_folder_path = data_folder_path
        # chdir(self.data_folder_path)  # FIXME
        self.rs = ResourceSet()
        self.mm: EPackage = _load_metamodel(self.rs, CBAUTIRO_ECORE)

    @ property
    def data_folder_name(self) -> str:
        return basename(self.data_folder_path)

    def is_inst(self, e_object: any, e_metaclass: MetaEClass):
        return isinstance(e_object, self.e_class(e_metaclass))

    def e_enum(self, e: EClassifier) -> EEnum:
        sub: EPackage
        for sub in self.mm.eSubpackages:
            if sub.name == e.ePackage.name:
                return sub.getEClassifier(e.name)

    def e_class(self, e: MetaEClass) -> EClass:
        e_class: EClass = e.eClass
        e_class_package: EPackage = e_class.ePackage

        if e_class_package.name == self.mm.name:
            return self.mm.getEClassifier(e_class.name)

        sub: EPackage
        for sub in self.mm.eSubpackages:
            if e_class_package.name == sub.name:
                return sub.getEClassifier(e_class.name)

    def get_abs_path(self, rel_path: str) -> str:
        """Usage inside DataService: Returns absolute path from relative(to <data_folder>)."""
        return join(self.data_folder_path, rel_path)

    def __get_resource(self, file_path: str) -> XMIResource:
        return self.rs.get_resource(file_path)

    def __create_resource(self, file_path) -> XMIResource:
        Path(dirname(file_path)).mkdir(parents=True, exist_ok=True)  # guaranty existence
        return self.rs.create_resource(file_path)

    def get_resource_rel(self, rel_path) -> XMIResource:
        """Use this to `ACCESS EXISTING` data-files (Workplans, Missions, Metadata, ...)."""
        return self.__get_resource(self.get_abs_path(rel_path))

    def create_resource_rel(self, rel_path) -> XMIResource:
        """Use this to `CREATE NEW` data-files (Workplans, Missions, Metadata, ...)."""
        return self.__create_resource(self.get_abs_path(rel_path))


def _load_metamodel(rs: ResourceSet, ecore_file: str) -> EPackage:
    """Creates, registers meta-model as Package (incl. Sub-Packages) from ecore and `RETURNS` mm."""

    metamodel_res: XMIResource = rs.get_resource(URI(ecore_file))
    metamodel: EPackage = metamodel_res.contents[0]

    # add Sub-Packages (= Sub-Metamodels) to registry
    rs.metamodel_registry[metamodel.nsURI] = metamodel
    for sub_mm in metamodel.eSubpackages:
        rs.metamodel_registry[sub_mm.nsURI] = sub_mm

    return metamodel


def get_metamodel_from(e: EObject = None,
                       res: XMIResource = None,
                       rs: ResourceSet = None) -> EPackage:
    """Util to navigate from any `EObject` to its MetaModel `EPackage`."""
    if e:
        return get_metamodel_from(res=e.eResource)
    if res:
        return get_metamodel_from(rs=res.resource_set)
    if rs:
        res_mm: XMIResource = rs.get_resource(URI(CBAUTIRO_ECORE))
        metamodel: EPackage = res_mm.contents[0]
        return metamodel
