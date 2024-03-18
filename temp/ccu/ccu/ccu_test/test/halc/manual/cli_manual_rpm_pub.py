#!/bin/python3
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from ccu_grpc.manual_mode_service_pb2_grpc import ManualModeServiceStub
from grpc import insecure_channel
from ccu_grpc.manual_mode_service_pb2 import DriveRpmControlMessage


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

            def rpm_drive_message_generator():
                for i in range(10):
                    print(i)
                    yield DriveRpmControlMessage(forward_backward_axis=0.85,
                                                 left_right_axis=0.25,
                                                 speed_mult=0.5)

            stub.DriveRpmControlStream(rpm_drive_message_generator())

    except KeyboardInterrupt:
        print(' --> MAIN stopped with CTRL-C')
        exit()


if __name__ == '__main__':
    start_client()
