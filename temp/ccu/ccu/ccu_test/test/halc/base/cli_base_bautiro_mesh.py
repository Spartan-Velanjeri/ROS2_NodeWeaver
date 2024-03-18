#!/bin/python3
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from sys import argv
from grpc import insecure_channel

from ccu_grpc.base_layer_service_pb2_grpc import BaseLayerServiceStub
from ccu_grpc.base_layer_service_pb2 import BautiroVisualFileRequest, FileStreamingMessage


# SERVER='192.168.0.2'
SERVER = 'localhost'
PORT = 50052


def start_client(file_path: str):
    from time import sleep
    from os.path import basename
    print('-------------------------------------------------------')
    print('TEST: ' + basename(__file__))
    print('-------------------------------------------------------')
    sleep(1)
    try:
        with insecure_channel(f'{SERVER}:{PORT}') as channel:
            stub = BaseLayerServiceStub(channel)

            print('GetBautiroMesh:')
            request = BautiroVisualFileRequest(file_path=file_path)
            msg: FileStreamingMessage
            for msg in stub.GetBautiroMesh(request):
                print(msg)

    except KeyboardInterrupt:
        print(' --> MAIN stopped with CTRL-C')
        exit()


if __name__ == '__main__':
    print(argv)
    if len(argv) > 1:
        start_client(argv[1])
    else:
        start_client('/etc/lsb-release')
