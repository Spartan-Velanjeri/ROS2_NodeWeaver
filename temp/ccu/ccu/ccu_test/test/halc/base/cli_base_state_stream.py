#!/bin/python3
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from time import time
from grpc import insecure_channel

from google.protobuf.empty_pb2 import Empty

from ccu_grpc.base_layer_service_pb2_grpc import BaseLayerServiceStub
from ccu_grpc.base_layer_service_pb2 import BaseLayerState


# SERVER='192.168.0.2'
SERVER = 'localhost'
PORT = 50052


def call(stub: BaseLayerServiceStub) -> None:
    print("call Iterator 'BaseLayerStateStream'")
    bls: BaseLayerState
    i = 0
    start_time = time()
    for bls in stub.BaseLayerStateStream(Empty()):
        i += 1
        print(f'···{i}··· ', end='')
        print(bls, end='')
        if time() - start_time > 15:
            break


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
            call(stub)

    except KeyboardInterrupt:
        print(' --> MAIN stopped with CTRL-C')
        exit()


if __name__ == '__main__':
    start_client()
