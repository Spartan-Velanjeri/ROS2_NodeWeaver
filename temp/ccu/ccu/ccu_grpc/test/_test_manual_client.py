#!/bin/python3
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from time import sleep
from grpc import insecure_channel

from ccu_grpc.manual_mode_service_pb2_grpc import ManualModeServiceStub
from ccu_grpc.manual_mode_service_pb2 import StartDrillMessage, StartDrillResponse
from ccu_grpc.base_layer_service_pb2 import DrillHole
from ccu_grpc.common_types_pb2 import Matrix4x4


def start_client():
    print('TEST: LAUNCH ManualMode Client: supporting only: StartDrill')
    try:
        with insecure_channel('localhost:50052') as channel:
            stub = ManualModeServiceStub(channel)

            sdm = StartDrillMessage(rotation_angle=1.1,
                                    drill_holes=[DrillHole(local_transform=Matrix4x4(r2c1=2.2,
                                                                                     r3c3=3.3),
                                                           name_unique_per_drillmask='hugo',
                                                           state=DrillHole.DrillHoleState.FINISHED,
                                                           parent_drillmask_name_unique='egon',
                                                           d_x=4.4,
                                                           d_y=5.5)])
            r: StartDrillResponse
            i = 0
            for r in stub.StartDrill(sdm):
                i += 1
                print(f'iterate {i}')
                which = r.WhichOneof('data')
                print(f'progress:  {r.progress:.2f}   \n' if 'progress' == which else '', end='')

                if 'finished' == which:
                    print(f'finished: {"success" if r.finished.success else "fail" }')

            print('CLIENT DONE  (sleep 10)')
            sleep(10.0)

    except KeyboardInterrupt:
        print(' --> MAIN stopped with CTRL-C')
        exit()


if __name__ == '__main__':
    start_client()
