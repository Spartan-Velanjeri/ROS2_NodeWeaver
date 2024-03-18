# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction
# editing, distribution, as well as in the event of applications for industrial
# property rights.


from os.path import abspath, dirname, join

from lxml import etree
from lxml.etree import XMLSchema, _Element, _ElementTree

# This calls must come together
# with following folders and files:
#     ├── <this_file>
#     └── xsd/
#         ├── DataTypes/
#         │   └── DataTypes-1.0/
#         │       └── DataTypes-1.0.xsd
#         ├── NodeTree/
#         │   └── NodeTree-2.0/
#         │       └── NodeTree-2.0.xsd
#         └── TaskPlan/
#             └── TaskPlan-2.1/
#                 └── TaskPlan-2.1.xsd


class TPValidator():
    """Validates `Taskplan.xml` files."""

    XSD = join(dirname(abspath(__file__)), 'xsd', 'TaskPlan', 'TaskPlan-2.1', 'TaskPlan-2.1.xsd')
    VERSION_2_1 = '2.1'
    SUPPORTED_VERSIONS = [VERSION_2_1]

    def __init__(self):
        self.xml_parser = etree.XMLParser(remove_blank_text=True)
        self.error_text = ''

    def validate(self, taskplan_path, xsd_file: str = XSD, tp_version: str = VERSION_2_1) -> bool:
        """
        Validates a <TaskPlan.xml> file (without Exception).
        Class-attribute `error_text` contains Exception Message after an invalid XMl was validated.

        :param taskplan_path: path to <TaskPlan.xml>
        :param xsd_file: (optional) path ro xsd file
        :param tp_version: Version of the <TaskPlan.xml>
        :return: True if valid, else False
        """
        schema: XMLSchema = etree.XMLSchema(
            etree.parse(xsd_file, self.xml_parser))
        try:
            e_tree = self._load_and_validate_for_xml(taskplan_path)
            self._check_version(e_tree, tp_version)
            self._validate_taskplan_schema(e_tree, schema)
            self.error_text = ''
        except Exception:
            return False
        return True

    def _load_and_validate_for_xml(self, taskplan_path) -> _ElementTree:
        """
        validates `taskplan_path` to be a valid XML, and loads into an `xml_element_tree`

        :param taskplan_path: path to taskplan file
        :raises Exception: in case taskplan is no valid XML
        :return: True if valid XML, else False
        """
        try:
            e_tree: _ElementTree = etree.parse(taskplan_path, self.xml_parser)
        except etree.XMLSyntaxError as error:
            self.error_text = 'File is not an XML: \n' + error.msg
            raise Exception(self.error_text)
        return e_tree

    def _check_version(self, e_tree: _ElementTree, tp_version: str) -> None:
        """
        Checks file is Version '2.0' TaskPlan.xml

        :param tp_version: version to be checked against (defaults to V_20)
        :param e_tree: xml to be checked if Version matches
        :raises Exception: in case it is not
        """
        if tp_version not in TPValidator.SUPPORTED_VERSIONS:
            self.error_text = f"TaskPlan Version {tp_version} is not supported"
            raise Exception()

        taskplan: _Element = e_tree.getroot()
        if 'version' not in taskplan.attrib:
            self.error_text = "TaskPlan.XML has no mandatory 'version' attribute"
            raise Exception()
        if tp_version != (v := taskplan.attrib['version']):
            self.error_text = f"TaskPlan file is version '{v}', but must be '{tp_version}'"
            raise Exception()

    def _validate_taskplan_schema(self, e_tree: _ElementTree, schema: XMLSchema) -> None:
        """
        validates `e_tree`(`=XML`) to be according to given xml-schema-definition

        :param e_tree: xml to be checked against scheme
        :param schema: xml-schema-definition that the xml shall be checked against
        :raises Exception: in case XML does not follow the scheme
        :return: True if scheme is followed, else False
        """
        assert e_tree is not None and schema is not None

        try:
            schema.assertValid(e_tree)
        except etree.DocumentInvalid:
            self.error_text = 'File not according to TaskPlan-XML Schema:'
            for error in schema.error_log:
                self.error_text += f"\n  Line {error.line}: {error.message}"
            raise Exception(self.error_text)


def validate_with_exception(taskplan_path,
                            xsd_file: str = TPValidator.XSD,
                            tp_version: str = TPValidator.VERSION_2_1) -> None:
    """
    Validates a <TaskPlan.xml> file and raise an Exception in case its invalid

    :param taskplan_path: path to <TaskPlan.xml>
    :param xsd_file: (optional) path ro xsd file
    :param tp_version: Version of the <TaskPlan.xml>
    :raises Exception: Exception in case of invalid. Has Text with what is wrong.
    """
    v = TPValidator()
    if v.validate(taskplan_path, xsd_file, tp_version):
        return
    raise Exception(v.error_text)
