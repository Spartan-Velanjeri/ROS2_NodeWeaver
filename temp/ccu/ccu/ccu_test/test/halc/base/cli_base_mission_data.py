#!/bin/python3
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from sys import argv
from grpc import insecure_channel

from ccu_grpc.base_layer_service_pb2_grpc import BaseLayerServiceStub
from ccu_grpc.base_layer_service_pb2 import MissionIdRequest, MissionDataResponse, Node


# SERVER='192.168.0.2'
SERVER = 'localhost'

PORT = 50052


def do_recurse(node: Node):
    print('NODE: ' + node.name_unique_per_node_tree)
    if node.fastener_template is not None:
        print('    FASTENER: ' + node.fastener_template.name_unique_per_node_tree)
    else:
        print('ARGFASTENER: ', end='')
        print(node.fastener_template)
    for n in node.children:
        do_recurse(n)


def start_client(index: int = 1):
    from time import sleep
    from os.path import basename
    print('-------------------------------------------------------')
    print('TEST: ' + basename(__file__))
    print('-------------------------------------------------------')
    sleep(1)
    try:
        with insecure_channel(f'{SERVER}:{PORT}') as channel:
            stub = BaseLayerServiceStub(channel)

            print('get_mission_data:')
            mdr: MissionDataResponse = stub.GetMissionData(MissionIdRequest(mission_id=index))
            print(mdr)
            # for node in mdr.node_tree:
            #     do_recurse(node)

    except KeyboardInterrupt:
        print(' --> MAIN stopped with CTRL-C')
        exit()


if __name__ == '__main__':
    print(argv)
    if len(argv) > 1:
        start_client(int(argv[1]))
    else:
        start_client(1)
