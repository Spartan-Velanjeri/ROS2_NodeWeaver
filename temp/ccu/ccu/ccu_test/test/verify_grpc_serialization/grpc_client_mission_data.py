#!/bin/python3
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from ccu_grpc.base_layer_service_pb2 import MissionIdRequest, StartMissionResponse
from ccu_grpc.base_layer_service_pb2_grpc import BaseLayerServiceStub
from grpc import insecure_channel


def start_client():
    try:
        with insecure_channel('localhost:50052') as channel:
            stub = BaseLayerServiceStub(channel)

            r: StartMissionResponse = stub.GetMissionData(MissionIdRequest())
            print(r)

    except KeyboardInterrupt:
        print(' --> MAIN stopped with CTRL-C')
        exit()


if __name__ == '__main__':
    start_client()
