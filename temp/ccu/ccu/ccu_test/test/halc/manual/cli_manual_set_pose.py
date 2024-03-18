#!/bin/python3
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from ccu_grpc.manual_mode_service_pb2_grpc import ManualModeServiceStub
from grpc import insecure_channel
from ccu_grpc.manual_mode_service_pb2 import SetStateMessage, SetStateMessageResponse  # noqa E501


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

            def call(value):
                for resp in stub.SetState(SetStateMessage(target_state=value)):
                    resp: SetStateMessageResponse
                    which = resp.WhichOneof('set_state_message_response_type')
                    obj = getattr(resp, which)
                    if (which == 'progress'):
                        print(f'Progress:{int(100 * obj.progress)} %')
                    else:
                        print(f'Final state success: {obj.success}')

            print("  Calling Set State FINE_POSITIONING   HCU call   CCU stream ")
            call(SetStateMessage.TargetState.FINE_POSITIONING)
            print("  Calling Set State DRIVING            HCU call   CCU stream ")
            call(SetStateMessage.TargetState.DRIVING)

    except KeyboardInterrupt:
        print(' --> MAIN stopped with CTRL-C')
        exit()


if __name__ == '__main__':
    start_client()
