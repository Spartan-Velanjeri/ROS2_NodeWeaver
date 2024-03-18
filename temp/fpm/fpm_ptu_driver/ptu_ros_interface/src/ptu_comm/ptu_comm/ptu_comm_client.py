"""
# -*- coding: utf-8 -*-
# Copyright (C) Robert Bosch GmbH 2022.
#
# All rights reserved, also regarding any disposal, exploitation,
# reproduction, editing, distribution, as well as in the event of
# applications for industrial property rights.
"""

"""
Basic funcionality based on example from:
https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html

Module description
------------------
--> see documentation in corresponding service module
"""

"""
__author__ = "Martin Dieterle (die2si)"
__license__ = "Bosch Internal Open Source License v4"
__email__ = "martin.dieterle@de.bosch.com"
__status__ = "Prototype" # Can be Prototype, Development, Production
"""
# Added after cloning bautiro_ros_interfaces
from bautiro_ros_interfaces.srv import FpmPtuComm    # Added 2022-10-17

import sys
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(FpmPtuComm, 'ptu_communication')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = FpmPtuComm.Request()

    def send_request(self, contr_int):
        self.req.contr_int = contr_int
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()
    # Instantiate client
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]))
    print('Response:', str(response))
    minimal_client.get_logger().info(
        'Result of ctrl_int %d ' % (int(sys.argv[1]))) 

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()