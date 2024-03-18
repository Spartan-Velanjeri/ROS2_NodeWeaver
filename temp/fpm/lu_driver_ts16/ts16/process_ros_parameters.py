#!/bin/python3
#
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

# flake8: noqa

def process_ros_parameters(Node,arg_list):
    for arg_i in arg_list:
        Node.declare_parameter(arg_i, '')
        arg_list[arg_i]["value"] = Node.get_parameter(arg_i).value
        if arg_list[arg_i]["value"] == '':
            arg_list[arg_i]["value"] = arg_list[arg_i]["default"]
            Node.get_logger().warn("Using default value for %s (%s)"%(arg_i,arg_list[arg_i]["value"]))

    return arg_list
