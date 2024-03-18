#! /usr/bin/env python3
# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

"""Moves arm to rest position."""

import time
import rclpy

from std_msgs.msg import Float64MultiArray

if __name__ == "__main__":
    rclpy.init()
    n = rclpy.create_node("arm_to_rest_node")
    log = n.get_logger()
    p = n.create_publisher(Float64MultiArray, "/forward_position_controller/commands", 1)
    while p.get_subscription_count() < 1:
        log.debug("Waiting for position controller")
        time.sleep(0.5)
        rclpy.spin_once(n)

    log.debug("Sending arm rest position")
    p.publish(
        Float64MultiArray(
            data=[
                -1.2667277495013636,
                -3.4994060001769007,
                -0.9075588583946228,
                1.2636422353931884,
                1.2625603675842285,
                0.0,
            ]
        )
    )
