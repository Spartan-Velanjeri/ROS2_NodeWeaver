# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from typing import Dict, List, Tuple, Callable
from types import FunctionType, ModuleType

from importlib import import_module
from pkgutil import walk_packages

from rclpy.node import Node
from bautiro_ros_interfaces.msg import Skill
from bautiro_ros_interfaces.srv import GetSkills
from ccu_grpc.base_layer_service_pb2 import (DeveloperFunctionList, DeveloperFunctionCallResult,
                                             DeveloperFunctionSource, DeveloperFunctionDefinition)
from ccu_util.util import dup_locate

from bautiro_developer_functions.core.call_srv import call_service_response
from bautiro_developer_functions.core.node_delegator import NodeDelegator
from bautiro_developer_functions.core.record import (DevFuncRecord, ErrorRecord,
                                                     SkillRecord, FncRecord)
import bautiro_developer_functions.py


class DeveloperFunctions(NodeDelegator):

    def __init__(self, node: Node):
        super().__init__(node)
        self.fh = FunctionHandler(node)
        self._records_cache: List[DevFuncRecord] = []

    def call_developer_function(self, dfd: DeveloperFunctionDefinition):
        message = f'No Function with name: {dfd.name}'
        for record in self._read_records_cache():
            if dfd.name == record.name:
                message = record.call(self.node)
        return DeveloperFunctionCallResult(message=message)

    def get_developer_functions(self) -> DeveloperFunctionList:
        self.log('GetDeveloperFunctions - called by client ... ')
        self._build_records_cache()
        sources = self._conv_records_to_sources()
        return DeveloperFunctionList(sources=sources)

    def _read_records_cache(self):
        if 0 == len(self._records_cache):
            self._build_records_cache()
        return self._records_cache

    def _build_records_cache(self) -> List[DevFuncRecord]:
        cache: List[DevFuncRecord] = []

        for sr in self.get_skill_records():
            cache.append(DevFuncRecord(source_name=sr.name, skill_record=sr))

        for fr in self.get_function_records():
            cache.append(DevFuncRecord(source_name=fr.mod, fnc_record=fr))

        if duplicates := dup_locate([r.name for r in cache]):
            cache.append(ErrorRecord(f'functions with same name: {duplicates}'))

        self._records_cache = cache
        return self._records_cache

    def _conv_records_to_sources(self) -> List[DeveloperFunctionSource]:
        src_fnc: Dict[str, List[DeveloperFunctions]] = dict()
        for r in self._records_cache:
            if r.source_name not in src_fnc:
                src_fnc[r.source_name] = []
            src_fnc[r.source_name].append(r.dev_func)
        return [DeveloperFunctionSource(name=s, functions=f) for s, f in src_fnc.items()]

    def get_skill_records(self) -> List[SkillRecord]:
        skill_tuples = self._get_server_skill_tuples()
        return [SkillRecord(srv, skill) for srv, skill in skill_tuples]

    def _get_server_skill_tuples(self) -> List[Tuple[str, Skill]]:
        server_skill_tuples = []
        for server_name in self.__get_server_names('bautiro_ros_interfaces/srv/GetSkills'):
            if r := call_service_response(self.node, GetSkills, server_name, GetSkills.Request()):
                for skill in r.skills:
                    server_skill_tuples.append((server_name, skill))
        return server_skill_tuples

    def __get_server_names(self, server_type: str) -> List[str]:
        names = []
        for name, types in self.node.get_service_names_and_types():
            if types and types[0] == server_type:
                names.append(name)
        self.log(f'Found {len(names)} GET_SKILLS server(s): {names}')
        return names

    def get_function_records(self) -> List[FncRecord]:
        return self.fh.get_function_records()


class FunctionHandler(NodeDelegator):

    def __init__(self, node: Node):
        super().__init__(node)

    def get_function_records(self) -> List[FncRecord]:
        fnc_tuples = self._get_mod_fnc_tuples(bautiro_developer_functions.py)
        self.log(f'Found {len(fnc_tuples)} Dev.-Functions.')
        [self.debug(f' mod: {m:>20} | fnc: {f.__name__:<30}') for m, f in fnc_tuples]
        return [FncRecord(mod=m, fnc=f) for m, f in fnc_tuples]

    def _get_mod_fnc_tuples(self, package: ModuleType) -> List[Tuple[str, Callable]]:
        result = []
        for _, sub_mod, _ in walk_packages(package.__path__, package.__name__+'.'):
            mod = import_module(sub_mod)
            for fnc in self._get_callables(mod):
                basename_submod = sub_mod.split('.')[-1]
                result.append((basename_submod, fnc))
        return result

    def _get_callables(self, mod: ModuleType) -> List[Callable]:

        def keep(s: str) -> bool:
            return not s.startswith('_') and s not in [
                "call_fire_and_forget",
                "call_fire_and_forget_bool",
                "call_str",
                "call_result",

                "call_action_fire_and_forget",
                "call_action_fire_and_forget_bool",
                "call_action_str",
                "call_action_result",

                "call_service_str",
                "call_service_response",

                "limit",
            ]
        return list(filter(lambda s: keep(s.__name__), self._get_all_callables(mod)))

    def _get_all_callables(self, mod: ModuleType) -> List[Callable]:
        callables = []
        for atr_name in mod.__dict__:
            a = getattr(mod, atr_name)
            if callable(a) and isinstance(a, FunctionType):
                callables.append(a)
        return callables
