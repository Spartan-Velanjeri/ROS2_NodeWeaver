# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from os.path import join
from ament_index_python.packages import get_package_share_directory

from rclpy import init, shutdown
from rclpy.executors import MultiThreadedExecutor
from ccu_dataservice.core import DSContext

from rclpy.node import Node
from ccu_data_services.services import bind_services


def main():
    init()
    n = Node('data_service')
    ctx = DSContext(logger=n.get_logger())
    n.get_logger().info(f"CCU-DATA-FOLDER: '{ctx.conf.data_folder_path}'")
    missions_dir = join(get_package_share_directory('ccu_data_services'), 'missions')
    bind_services(n, ctx, missions_dir)
    try:
        exe = MultiThreadedExecutor(10)
        exe.add_node(n)
        exe.spin()
    except KeyboardInterrupt:
        print('Data-Service stopped with CTRL-C')
    finally:
        n.destroy_node()
        shutdown()


if __name__ == '__main__':
    main()
