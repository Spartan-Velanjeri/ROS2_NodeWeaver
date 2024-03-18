# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from threading import Thread
from concurrent.futures import ThreadPoolExecutor
from rclpy import init, shutdown
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from grpc import server as create_server
from ccu_grpc.manual_mode_service_pb2_grpc import add_ManualModeServiceServicer_to_server
from ccu_grpc.base_layer_service_pb2_grpc import add_BaseLayerServiceServicer_to_server

from ccu_hcu_abstraction.base.srv import BaseSrv
from ccu_hcu_abstraction.manual.srv import ManualSrv


###################################################################################################
#                                                                                                 #
#  main: grpc-server and ros-node in separate threads                                             #
#                                                                                                 #
###################################################################################################

def grpc_serve(manual_srv: ManualSrv, base_srv: BaseSrv):
    server = create_server(ThreadPoolExecutor(12))
    add_ManualModeServiceServicer_to_server(manual_srv, server)
    add_BaseLayerServiceServicer_to_server(base_srv, server)
    port = 50052
    base_srv.df.log(f"grpc-server listening on port {port}")
    server.add_insecure_port(f'[::]:{port}')
    server.start()
    server.wait_for_termination()


def main():
    """Launch grpc-server and ros-node in separate threads, bridge call-backs."""
    init()
    node = Node('halc')
    node.get_logger().info('Launch halc')

    exe = MultiThreadedExecutor(24)
    exe.add_node(node)
    t = Thread(target=grpc_serve, args=(ManualSrv(node), BaseSrv(node)))
    try:
        t.start()
        exe.spin()
        t.join()
    except KeyboardInterrupt:
        print('halc stopped with CTRL-C')
        exe.remove_node(node)
    shutdown()


if __name__ == '__main__':
    main()
