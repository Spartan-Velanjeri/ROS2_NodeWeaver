#!/bin/python3
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from grpc import insecure_channel
from google.protobuf.empty_pb2 import Empty
from ccu_grpc.base_layer_service_pb2_grpc import BaseLayerServiceStub
from ccu_grpc.base_layer_service_pb2 import DeveloperFunctionList, DeveloperFunctionDefinition,  \
    ParameterDefinition, NumericParameter, BoolParameter


"""
set_state_rpm_driving           |
set_state_rpm_fine_positioning  |
hu_move_relative                | ['number: x', 'number: y']
start_drill                     | ['number: depth']
move_lift_to_height             | ['number: target_height']
start_feinlokalisierung         |
set_active_mission              | ['number: mission_index_to_set']
unset_active_mission            |
start_ccu_bt                    | ['number: bt_xml_enum']
ptu_an                          |
ptu_aus                         |
ptu_ansteuern                   | ['number: drehzahl', 'bool: rechts_drehend']
staubsauger_an                  |
staubsauger_aus                 |
manuelles_homing                |
next_grobpose_anfahren          |
grobpose_n_anfahren             | ['number: cluster_index']
rpm_1_sec_geradeaus_fahren      |
rpm_1_sec_in_richtung_fahren    | ['number: translation', 'number: rotation', '
"""
SERVER = 'localhost'  # SERVER='192.168.0.2'
PORT = 50052


def get_function(name: str, f_list: DeveloperFunctionList) -> DeveloperFunctionDefinition:
    for source in f_list.sources:
        for f in source.functions:
            if name == f.name:
                return f


def print_info(f: DeveloperFunctionDefinition):
    print('=========  ' + f.name + '  ============')
    for p in f.parameters:
        print(p)


def call(stub: BaseLayerServiceStub, f: DeveloperFunctionDefinition) -> None:
    response_message = stub.CallDeveloperFunction(f)
    print('--------------------------------------------------------')
    print(response_message)


def start_client():
    from time import sleep
    from os.path import basename
    print('-------------------------------------------------------')
    print('TEST: ' + basename(__file__))
    print('-------------------------------------------------------')
    sleep(1)
    try:
        with insecure_channel(f'{SERVER}:{PORT}') as channel:
            stub = BaseLayerServiceStub(channel)
            f_list: DeveloperFunctionList = stub.GetDeveloperFunctions(Empty())

            print_info(get_function('ptu_ansteuern', f_list))
            print_info(get_function('set_active_mission', f_list))

            call(stub, DeveloperFunctionDefinition(
                name='ptu_ansteuern',
                parameters=[
                    ParameterDefinition(param_number=NumericParameter(
                        param_name='drehzahl',
                        param_value=2342.0)),
                    ParameterDefinition(param_bool=BoolParameter(
                        param_name='rechts_drehend',
                        param_value=False)),]))

            call(stub, DeveloperFunctionDefinition(
                name='set_active_mission',
                parameters=[
                    ParameterDefinition(param_number=NumericParameter(
                        param_name='mission_index_to_set',
                        param_value=1.0)),]))

    except KeyboardInterrupt:
        print(' --> MAIN stopped with CTRL-C')
        exit()


if __name__ == '__main__':
    start_client()
