#!/bin/python3
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.


from grpc import insecure_channel
from google.protobuf.empty_pb2 import Empty
from ccu_grpc.base_layer_service_pb2_grpc import BaseLayerServiceStub
from ccu_grpc.base_layer_service_pb2 import (DeveloperFunctionList, ParameterDefinition,
                                             DeveloperFunctionDefinition, DeveloperFunctionSource )


# SERVER='192.168.0.2'
SERVER = 'localhost'

PORT = 50052


def my_form(p: ParameterDefinition):
    name_num = p.param_number.param_name
    name_bool = p.param_bool.param_name
    return f'number: {name_num}' if name_num else f'bool: {name_bool}'


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

            print('\n call  stub.GetDeveloperFunctions()\n')
            print(f"{'sources':>20} | {'functions':<30}| {'arguments'} ")
            print(f"{'---------------':>20} | {'------------------------':<30}| {'---------------------'} ")  # noqa
            fnc_list: DeveloperFunctionList = stub.GetDeveloperFunctions(Empty())
            si = 0
            for s in fnc_list.sources:
                s: DeveloperFunctionSource
                si += 1
                fi = 0
                for f in s.functions:
                    f: DeveloperFunctionDefinition
                    fi += 1
                    arguments = [my_form(p) for p in f.parameters]
                    if 0 == len(arguments):
                        arguments = ''
                    print(f"{s.name:>20} | {f.name:<30}| {arguments} ")
            print(f"{'---------------':>20} | {'------------------------':<30}| {'---------------------'} ")  # noqa

    except KeyboardInterrupt:
        print(' --> MAIN stopped with CTRL-C')
        exit()


if __name__ == '__main__':
    start_client()
