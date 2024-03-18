#!/bin/python3
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.


from grpc import insecure_channel

from ccu_grpc.common_types_pb2 import Matrix4x4

from ccu_grpc.manual_mode_service_pb2_grpc import ManualModeServiceStub
from ccu_grpc.manual_mode_service_pb2 import StartDrillMessage, StartDrillResponse


def start_client():
    from time import sleep
    from os.path import basename
    print('-------------------------------------------------------')
    print('TEST: ' + basename(__file__))
    print('-------------------------------------------------------')
    sleep(1)
    try:
        with insecure_channel('localhost:50052') as channel:

            i = 0
            r: StartDrillResponse
            for r in ManualModeServiceStub(channel).StartDrill(
                    StartDrillMessage(drill_holes=[
                        StartDrillMessage.ManualDrillHole(
                            local_transform=Matrix4x4(r1c1=1.1, r4c4=1.0),
                            name='hugo',
                            depth_meter=0.035,
                            diameter_meter=0.008)
                    ])):
                i += 1
                print(f'iterate {i}')
                which = r.WhichOneof('data')
                print(f'progress:  {r.progress:.2f}   \n' if 'progress' == which else '', end='')
                print(f'finished: {r.finished.success}\n' if 'finished' == which else '', end='')

    except KeyboardInterrupt:
        print(' --> MAIN stopped with CTRL-C')
        exit()


if __name__ == '__main__':
    start_client()
