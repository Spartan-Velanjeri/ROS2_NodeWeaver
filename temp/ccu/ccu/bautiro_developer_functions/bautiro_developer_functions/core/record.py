# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from typing import List, Tuple
from inspect import signature, Parameter
from rclpy.node import Node
from ccu_grpc.base_layer_service_pb2 import (DeveloperFunctionDefinition as DfD,
                                             ParameterDefinition, NumericParameter, BoolParameter)
from bautiro_ros_interfaces.msg import Skill, SkillInput
from typing import Callable


class FncRecord():

    SUPPORTED_TYPES = [float, bool, int]

    def __init__(self,
                 mod: str,
                 fnc: Callable,
                 ):
        self.fnc = fnc
        self.mod = mod
        self._proto_native_mapping = []

    @property
    def name(self):
        return self.fnc.__name__

    def to_dev_func(self) -> DfD:
        return DfD(name=self.name,
                   description=self.name,
                   parameters=[pa[0] for pa in self.proto_native_mapping])

    @property
    def proto_native_mapping(self) -> List[Tuple[ParameterDefinition, Tuple[str, type, any]]]:
        if not self._proto_native_mapping:
            self._proto_native_mapping = self._convert_all_triples()
        return self._proto_native_mapping

    def _convert_all_triples(self) -> List[Tuple[ParameterDefinition, Tuple[str, type, any]]]:
        mapping = []
        for n, t, d in self._get_native_arguments_triples():
            proto = self._argument_2_parameter(n, t, d)
            native = (n, t, d)
            mapping.append((proto, native))
        return mapping

    def _argument_2_parameter(self, name: str, t: type, default_value: any):
        if bool == t:
            return ParameterDefinition(param_bool=BoolParameter(
                param_name=name,
                param_value=default_value or False))
        # else int or float
        step_size = 1.0
        min_value = -111111.1
        max_value = 222222.2
        return ParameterDefinition(param_number=NumericParameter(
            param_name=name,
            param_value=default_value or 0.0,
            min_value=min_value,
            max_value=max_value,
            step_size=step_size))

    def _get_native_arguments_triples(self) -> List[Tuple[str, type, any]]:
        """Return list of Triples as [arg_name, arg_type, arg_defaultvalue]."""
        name_type_defaults = []
        first = True
        p: Parameter
        for arg_name, p in signature(self.fnc).parameters.items():
            arg_type = p.annotation
            if first:
                self.validate_first(self.name, arg_name, arg_type)
                first = False
                continue
            arg_default = p.default if p.default is not Parameter.empty else None
            if arg_type in FncRecord.SUPPORTED_TYPES:
                name_type_defaults.append((arg_name, arg_type, arg_default))
            else:
                self.validate_arg(self.name, arg_name, arg_type)
        return name_type_defaults

    def validate_first(self, fnc_name: str, arg_name: str, arg_type: type):
        if arg_type == Node:
            return
        t = arg_type.__name__
        raise TypeError(f"first argument must be type 'Node' - {fnc_name}({arg_name}: {t})")

    def validate_arg(self, fnc_name: str, arg_name: str, arg_type: type):
        t = arg_type.__name__
        raise TypeError(F"Only 'float' and 'int' allowed - {fnc_name}({arg_name}: {t})")

    def call(self, node: Node) -> str:
        try:
            txt = self.fnc(node)
            return txt
        except BaseException as e:
            return "EXCEPTION: " + str(e)


class SrvRecord():

    def __init__(self,
                 server_name: str = None,
                 srv_type=None,
                 ):
        self.server_name = server_name
        self.srv_type = srv_type

    @property
    def name(self):
        return self.server_name

    def to_dev_func(self) -> DfD:
        return DfD()

    def call(self, node: Node) -> str:
        try:
            return "Not Implemented (SrvRecord)"
        except BaseException as e:
            return "EXCEPTION: " + str(e)


class SkillRecord():

    def __init__(self,
                 skill_server_name: str = None,
                 skill: Skill = None,
                 ):
        self.skill_server_name = skill_server_name
        self.skill = skill

    @property
    def name(self):
        return self.skill_server_name

    def call(self, node: Node):
        return 'Not Yet Implemented (SkillRecord.call)'

    def to_dev_func(self) -> DfD:
        def _input_to_param(input: SkillInput) -> ParameterDefinition:
            if input.type.lower() == 'float':
                return ParameterDefinition(param_number=_float_to_numeric(input))
            if input.type.lower() == 'int':
                return ParameterDefinition(param_number=_int_to_numeric(input))
            if input.type.lower() == 'bool':
                return ParameterDefinition(param_bool=_bool_to_bool(input))
            if input.type.lower() == 'string':
                return ParameterDefinition(param_bool=_string_to_bool(input))

        def _int_to_numeric(input: SkillInput) -> NumericParameter:
            return _float_to_numeric(input)

        def _bool_to_bool(input: SkillInput) -> BoolParameter:
            return BoolParameter(param_name=input.name,
                                 param_value=True)  # TODO no default-value in SkillInput

        def _string_to_bool(input: SkillInput) -> BoolParameter:
            return BoolParameter(param_name='STRING: ' + input.name,
                                 param_value=True)   # TODO no STRING support in Param-Definition

        def _float_to_numeric(input: SkillInput) -> NumericParameter:
            step_size = 1.0
            param_value = 1.0
            min_value = float(input.min)
            max_value = float(input.max)
            param_name = input.name
            return NumericParameter(param_name=param_name, param_value=param_value,
                                    min_value=min_value, max_value=max_value, step_size=step_size)

        return DfD(name=self.skill.name,
                   description=self.skill.description,
                   parameters=[_input_to_param(i) for i in self.skill.inputs])


class DevFuncRecord():

    def __init__(self,
                 source_name: str,
                 skill_record: SkillRecord = None,
                 fnc_record: FncRecord = None,
                 srv_record: SrvRecord = None
                 ):
        self.source_name = source_name
        self.skill_record = skill_record
        self.srv_record = srv_record
        self.fnc_record = fnc_record

        if skill_record:
            self.dev_func = skill_record.to_dev_func()
        if srv_record:
            self.dev_func = srv_record.to_dev_func()
        if fnc_record:
            self.dev_func = fnc_record.to_dev_func()

    @property
    def name(self):
        return self.dev_func.name

    def call(self, node: Node) -> str:
        if self.skill_record:
            return self.skill_record.call(node)
        if self.srv_record:
            return self.srv_record.call(node)
        if self.fnc_record:
            return self.fnc_record.call(node)


class ErrorRecord(DevFuncRecord):

    def __init__(self, message: str):
        super().__init__(source_name='Errors')
        self.message = message

    @property
    def name(self):
        return self.message
