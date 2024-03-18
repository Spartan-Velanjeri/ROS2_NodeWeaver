# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from rclpy.node import Node


class NodeDelegator():

    def __init__(self, node: Node):
        self.node = node

    def log(self, txt: str) -> str:
        return self.info(txt)

    def debug(self, txt: str) -> str:
        txt = txt if isinstance(txt, str) else repr(txt)
        if self.node:
            self.node.get_logger().debug(txt)
        else:
            print(txt)
        return txt

    def info(self, txt: str) -> str:
        txt = txt if isinstance(txt, str) else repr(txt)
        if self.node:
            self.node.get_logger().info(txt)
        else:
            print(txt)
        return txt

    def warn(self, txt: str) -> str:
        txt = txt if isinstance(txt, str) else repr(txt)
        if self.node:
            self.node.get_logger().warn(txt)
        else:
            print(txt)
        return txt

    def error(self, txt: str) -> str:
        txt = txt if isinstance(txt, str) else repr(txt)
        if self.node:
            self.node.get_logger().error(txt)
        else:
            print(txt)
        return txt

    def fatal(self, txt: str) -> str:
        txt = txt if isinstance(txt, str) else repr(txt)
        if self.node:
            self.node.get_logger().fatal(txt)
        else:
            print(txt)
        return txt
