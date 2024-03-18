#!/bin/python3

# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights
from ccu_interfaces.srv import ProtoInRos
from rclpy import init, shutdown, spin_until_future_complete, create_node

from ccu_grpc.util import to_str, inst
from ccu_grpc.base_layer_service_pb2 import (MissionIdRequest,
                                             MissionDataResponse)


def call_proto_in_ro_service(proto_req_msg, proto_response_typ):
    init()
    node = create_node('minimal_client')
    cli = node.create_client(ProtoInRos, 'get_mission_data')
    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')
    future = cli.call_async(ProtoInRos.Request(proto=to_str(proto_req_msg)))
    spin_until_future_complete(node, future)
    node.destroy_node()
    shutdown()
    resp: ProtoInRos.Response = future.result()
    return inst(proto_response_typ, resp.proto)


def main():

    for i in range(1, 2):
        q  = MissionIdRequest(mission_id=i)
        r = call_proto_in_ro_service(proto_req_msg=q,
                                     proto_response_typ=MissionDataResponse)
        print(f'{i}--{len(repr(r))}')


if __name__ == '__main__':
    main()
