#!/bin/python3
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from concurrent.futures import ThreadPoolExecutor
from time import sleep
from grpc import server as create_server

from ccu_grpc.manual_mode_service_pb2 import StartDrillMessage, StartDrillResponse, DrillFinishedMessage
from ccu_grpc.manual_mode_service_pb2_grpc import (ManualModeServiceServicer,
                                            add_ManualModeServiceServicer_to_server)


class ManualModeSrvTest(ManualModeServiceServicer):

    def StartDrill(self, q: StartDrillMessage, context):
        print(f"""
Server Received:  - StartDrillMessage
                          - rotation_angle: {q.rotation_angle:.2f}
                          - drillholes:     {len(q.drill_holes)}""")
        for dh in q.drill_holes:
            print(f"""
          name:  {dh.name_unique_per_drillmask}
          parent:{dh.parent_drillmask_name_unique}
          state: {dh.state}
          d_x:   {dh.d_x}
          d_y:   {dh.d_y}
local_transform: {dh.local_transform}""")

        def server_response_stream_generator():
            print('Server Response  PROGRESS: ', end='')
            for i in range(1, 7):
                sleep(0.3)
                print(f'{i},', end='')
                yield StartDrillResponse(progress=i/7)
            print('\n')
            yield StartDrillResponse(finished=DrillFinishedMessage(success=True))
            print('Server Response  FINISHED: True')
            return

        return server_response_stream_generator()


def grpc_serve():
    print('TEST: LAUNCH ManualMode SERVER: supporting only: StartDrill')
    server = create_server(ThreadPoolExecutor())
    add_ManualModeServiceServicer_to_server(ManualModeSrvTest(), server)
    server.add_insecure_port('[::]:50052')
    server.start()
    server.wait_for_termination()


if __name__ == '__main__':
    grpc_serve()
