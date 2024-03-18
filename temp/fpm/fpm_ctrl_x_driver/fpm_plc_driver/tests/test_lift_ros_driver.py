# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction
# editing, distribution, as well as in the event of applications for industrial
# property rights.

"""Tests for the lift ROS node."""

import pytest
import rclpy
from bautiro_ros_interfaces.action import MoveLiftAbsolute
from bautiro_ros_interfaces.srv import GetLiftPosition, SetLiftVelocityProfile
from rclpy.action import ActionClient
from rclpy.node import Node


@pytest.fixture
def set_velocity_profile():
    """Fixture for set velocity service."""

    def _set_velocity_client(node: Node, velocity_profile: float):
        """Checking set veloecity service."""
        test_client = node.create_client(SetLiftVelocityProfile, "/lift_velocity_set")
        node.get_logger().info("Created test client.")

        assert test_client.wait_for_service(timeout_sec=2.0)
        node.get_logger().info("Test client ready.")
        request = SetLiftVelocityProfile.Request()
        request.lift_profile_velocity = velocity_profile
        node.get_logger().info("Sending request ...")
        future = test_client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=2)
        node.get_logger().info("Request completed.")
        try:
            future.result()
        except BaseException:
            assert False, "Velocity set failed."
        node.get_logger().info(f"Result received: {future.result()}")
        assert not future.result().response_code.data

    return _set_velocity_client


@pytest.fixture
def get_position():
    """Asserting get position service."""

    def _get_position(node: Node):
        position_client = node.create_client(GetLiftPosition, "/get_lift_position")
        node.get_logger().info("Created position client.")

        assert position_client.wait_for_service(timeout_sec=2.0)
        node.get_logger().info("Position client ready.")
        request = GetLiftPosition.Request()
        node.get_logger().info("Sending position request ...")
        future = position_client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=2)
        node.get_logger().info("Position request completed.")
        try:
            future.result()
        except BaseException:
            assert False, "Position reading failed."
        node.get_logger().info(f"Result received: {future.result()}")

        return future.result().lift_position

    return _get_position


# ############# Test publishers
@pytest.mark.skip(reason="Not implemented")
def test_lift_position():
    pass


@pytest.mark.skip(reason="Not implemented")
def test_topic_lift_state():
    pass


@pytest.mark.skip(reason="Not implemented")
def test_tf():
    pass


# ############# Test services

@pytest.mark.skip(reason="Not implemented")
def test_get_lift_position():
    """Test for retreiving lift position."""
    # call nodes end-point
    rclpy.init()
    node = rclpy.create_node("test_node")
    node.get_logger().info("Starting test_node ...")
    # position_client = node.create_client(GetLiftPosition, "/get_lift_position")
    # node.get_logger().info("Created position client.")

    # assert position_client.wait_for_service(timeout_sec=2.0)
    # node.get_logger().info("Position client ready.")
    # request = GetLiftPosition.Request()
    # node.get_logger().info("Sending request ...")
    # future = position_client.call_async(request)
    # rclpy.spin_until_future_complete(node, future, timeout_sec=2)
    # node.get_logger().info("Request completed.")
    # try:
    #     future.result()
    # except BaseException:
    #     assert False, "Position reading failed."
    # node.get_logger().info(f"Result received: {future.result()}")

    node.destroy_node()
    rclpy.shutdown()


@pytest.mark.skip(reason="Not implemented")
def test_lift_error_quit():
    """Test for quiting error."""
    pass


@pytest.mark.skip(reason="Not implemented")
def test_lift_velocity_set():
    """Test for setting lift velocity profile."""

    rclpy.init()
    node = rclpy.create_node("test_node")
    node.get_logger().info("Starting test_node ...")
    test_client = node.create_client(SetLiftVelocityProfile, "/lift_velocity_set")
    node.get_logger().info("Created test client.")

    assert test_client.wait_for_service(timeout_sec=2.0)
    node.get_logger().info("Test client ready.")
    request = SetLiftVelocityProfile.Request()
    request.lift_profile_velocity = 0.01
    node.get_logger().info("Sending request ...")
    future = test_client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=2)
    node.get_logger().info("Request completed.")
    try:
        future.result()
    except BaseException:
        assert False, "Velocity set failed."
    node.get_logger().info(f"Result received: {future.result()}")
    assert not future.result().response_code.data
    node.destroy_node()
    rclpy.shutdown()


@pytest.mark.skip(reason="Not implemented")
def test_move_lift_absolute_stop():
    pass


# ############# Test actions


@pytest.mark.skip(reason="Not implemented")
def test_move_lift_absolute(set_velocity_profile, get_position):
    """Testing of motion driver parameters and functions:
    * in_motion - flag
    * move_lift_absolute - fcn
    * lift_position - parameter
    * lift_position_meter - parameter
    """
    const_velocity_profile = 0.05
    rclpy.init()
    node = rclpy.create_node("test_node")
    node.get_logger().info("Starting test_node ...")
    # use small velocity
    set_velocity_profile(node, const_velocity_profile)
    print(f"\tNew velocity profile [mm/sec]: {const_velocity_profile}")
    current_position = get_position(node)
    print(f"\tCurrent position [m]: {current_position}")
    if current_position > 0.050:
        new_position = current_position - 0.050
    else:
        new_position = current_position + 0.050

    node._ac = ActionClient(
        node, action_type=MoveLiftAbsolute, action_name="/move_lift_absolute"
    )

    goal = MoveLiftAbsolute.Goal()
    goal.requested_target_lift_level = new_position

    assert node._ac.wait_for_server(
        timeout_sec=2
    ), "/move_lift_absolute server not ready"

    # client.move_lift_absolute(position_setpoint=new_position)
    # print(f"\tMoving to position [mm]: {new_position}")

    # time.sleep(0.5)

    # assert client.in_motion

    # while client.in_motion:
    #     time.sleep(0.5)
    #     print(f"\tVelocity [mm/sec]: {client.lift_velocity}")
    #     # assert client.lift_velocity == const_velocity_profile

    # print(f"\tNew position [mm]: {client.lift_position}")
    # assert client.lift_position == new_position
    # print(f"\tNew position [m]: {client.lift_position_meter}")
    # assert client.lift_position_meter == new_position / 1000
