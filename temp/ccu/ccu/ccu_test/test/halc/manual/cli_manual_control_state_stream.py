#!/bin/python3
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from ccu_grpc.manual_mode_service_pb2_grpc import ManualModeServiceStub
from grpc import insecure_channel
from google.protobuf.empty_pb2 import Empty
from ccu_grpc.manual_mode_service_pb2 import ControlState
from time import time


def start_client():
    from time import sleep
    from os.path import basename
    print('-------------------------------------------------------')
    print('TEST: ' + basename(__file__))
    print('-------------------------------------------------------')
    sleep(1)
    try:
        with insecure_channel('localhost:50052') as channel:
            stub = ManualModeServiceStub(channel)

            print("------------------------ ControlState                  <---- DOWN  (stream infinite == 10x )")  # noqa E501
            s: ControlState
            start_time = time()
            for s in stub.ControlStateStream(Empty()):
                print(f'can_drive_rpm: {s.can_drive_rpm}')
                print(f'can_lower_lift: {s.can_lower_lift}')
                print(f'can_move_arm: {s.can_move_arm}')
                print(f'can_raise_lift: {s.can_raise_lift}')
                if time() - start_time > 15:
                    break

    except KeyboardInterrupt:
        print(' --> MAIN stopped with CTRL-C')
        exit()


if __name__ == '__main__':
    start_client()
