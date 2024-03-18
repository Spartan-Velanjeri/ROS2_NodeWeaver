#!/bin/python3
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from sys import argv
from grpc import insecure_channel

from ccu_grpc.base_layer_service_pb2_grpc import BaseLayerServiceStub
from ccu_grpc.base_layer_service_pb2 import MissionIdRequest, StartMissionResponse


# To Test:
#  - python3 t_backend_node_AS_StartCcuBt.py
#  - ros2 run ccu_data_services main
#  - ros2 run ccu_hcu_abstraction main
#  - t_grpc-client_base_start_mission.py


def start_client(mission_id):
    from time import sleep
    from os.path import basename
    print('-------------------------------------------------------')
    print('TEST: ' + basename(__file__))
    print('-------------------------------------------------------')
    sleep(1)
    try:
        with insecure_channel('localhost:50052') as channel:

            print('start_mission:')
            stub = BaseLayerServiceStub(channel)

            r1: StartMissionResponse = stub.StartMission(MissionIdRequest(mission_id=mission_id))
            print(f'Start Mission [ID: {mission_id}] returned "success"-state: {r1.success}')

    except KeyboardInterrupt:
        print(' --> MAIN stopped with CTRL-C')
        exit()


if __name__ == '__main__':
    print(argv)
    if len(argv) > 1:
        start_client(int(argv[1]))
    else:
        start_client(1)
