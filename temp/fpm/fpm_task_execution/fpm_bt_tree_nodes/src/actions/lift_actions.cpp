// Copyright (c) 2024 Robert Bosch GmbH and its subsidiaries
//
// All rights reserved, also regarding any disposal, exploitation, reproduction,
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#include "fpm_bt_tree_nodes/actions/lift_actions.hpp"

namespace fpm_bt_tree_nodes {

//------------------------LiftMoveAbsolute-------------------------------
LiftMoveAbsolute::LiftMoveAbsolute(const std::string &name,
                                   const BT::NodeConfiguration &conf)
    : BT::ActionNodeBase(name, conf) {
  // getInput("value", initial);
}

inline BT::NodeStatus LiftMoveAbsolute::tick() {

  rclcpp::Node::SharedPtr node_;
  node_ = rclcpp::Node::make_shared("lift_move_absolute");

  node_->declare_parameter("client_wait_time", 2);

  // Create action client for the lift driver service
  auto action_client = rclcpp_action::create_client<
      bautiro_ros_interfaces::action::MoveLiftAbsolute>(node_,
                                                        "/move_lift_absolute");

  if (!action_client->wait_for_action_server(std::chrono::seconds(
          node_->get_parameter("client_wait_time").as_int()))) {
    RCLCPP_ERROR(node_->get_logger(), "Lift driver service not available.");
    return BT::NodeStatus::FAILURE;
  }

  // send target lift level
  auto goal_msg = bautiro_ros_interfaces::action::MoveLiftAbsolute::Goal();
  RCLCPP_INFO(node_->get_logger(), "Mesage after init: %f",
              goal_msg.requested_target_lift_level);

  auto lift_setpoint_position = getInput<float>(
      "lift_setpoint_position", goal_msg.requested_target_lift_level);

  if (!lift_setpoint_position.has_value()) {
    throw BT::RuntimeError("Missing required input [lift_setpoint_position]: ",
                           lift_setpoint_position.error());
  }

  auto goal_handle_future = action_client->async_send_goal(goal_msg);
  if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {

    RCLCPP_ERROR(node_->get_logger(), "Sending goal call failed.");
    return BT::NodeStatus::FAILURE;
  }

  rclcpp_action::ClientGoalHandle<
      bautiro_ros_interfaces::action::MoveLiftAbsolute>::SharedPtr goal_handle =
      goal_handle_future.get();
  if (!goal_handle) {

    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    return BT::NodeStatus::FAILURE;
  }

  auto result_future = action_client->async_get_result(goal_handle);

  if (rclcpp::spin_until_future_complete(node_, result_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Failure in obtaining result.");
    return BT::NodeStatus::FAILURE;
  }

  rclcpp_action::ClientGoalHandle<
      bautiro_ros_interfaces::action::MoveLiftAbsolute>::WrappedResult
      wrapped_result = result_future.get();


  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED){
    // action is failed
    RCLCPP_ERROR(node_->get_logger(),
              "Lift driver: Result code: %d. Lift level: %f,",
              wrapped_result.result->response_code,
              wrapped_result.result->final_lift_level);
    return BT::NodeStatus::FAILURE;
  } 

  RCLCPP_INFO(node_->get_logger(),
              "Lift driver: Result code: %d. Lift level: %f,",
              wrapped_result.result->response_code,
              wrapped_result.result->final_lift_level);

  return BT::NodeStatus::SUCCESS;
}

//------------------------LiftMoveToWorkingHeight-------------------------------

LiftMoveToWorkingHeight::LiftMoveToWorkingHeight(
    const std::string &name, const BT::NodeConfiguration &conf)
    : BT::ActionNodeBase(name, conf) {
  // getInput("value", initial);
}

inline BT::NodeStatus LiftMoveToWorkingHeight::tick() {

  rclcpp::Node::SharedPtr node_;
  node_ = rclcpp::Node::make_shared("lift_move_to_working_pose");

  node_->declare_parameter<int>("client_wait_time", 2);             // unit [s]
  node_->declare_parameter<float>("platform_height", 1.62);         // unit [m]
  node_->declare_parameter<float>("platform_to_robot_base", 0.095); // unit [m]
  node_->declare_parameter<float>("optimal_tcp_level", 0.635);      // unit [m]
  node_->declare_parameter<float>("ptu_length", 0.545);             // unit [m]
  node_->declare_parameter<float>("working_distance", 0.02);        // unit [m]

  // Create action client for the lift driver service
  auto lift_move_action_client = rclcpp_action::create_client<
      bautiro_ros_interfaces::action::MoveLiftAbsolute>(node_,
                                                        "/move_lift_absolute");

  if (!lift_move_action_client->wait_for_action_server(std::chrono::seconds(
          node_->get_parameter("client_wait_time").as_int()))) {
    RCLCPP_ERROR(node_->get_logger(), "Lift driver service not available.");
    return BT::NodeStatus::FAILURE;
  }

  // get current lift level
  float lift_current_position;
  auto lift_position_client =
    node_->create_client<bautiro_ros_interfaces::srv::GetLiftPosition>("get_lift_position");

  if (!lift_position_client ->wait_for_service(std::chrono::seconds(
          node_->get_parameter("client_wait_time").as_int()))) {
    RCLCPP_ERROR(node_->get_logger(), "Lift driver service not available.");
    return BT::NodeStatus::FAILURE;
  } 

  auto request = std::make_shared<bautiro_ros_interfaces::srv::GetLiftPosition::Request>();

  auto result = lift_position_client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node_, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    lift_current_position = result.get()->lift_position;
    RCLCPP_INFO(node_->get_logger(), "Received lift position: %f", lift_current_position);
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Failed to obtain lift position.");
    return BT::NodeStatus::FAILURE;
  }

  // send target lift level
  auto goal_msg = bautiro_ros_interfaces::action::MoveLiftAbsolute::Goal();
  RCLCPP_INFO(node_->get_logger(), "Mesage after init: %f",
              goal_msg.requested_target_lift_level);

  float platform_ceiling_distance;
  auto platform_ceiling_distance_handle =
      getInput<float>("platform_ceiling_distance", platform_ceiling_distance);

  if (!platform_ceiling_distance_handle.has_value()) {
    throw BT::RuntimeError(
        "Missing required input [platform_ceiling_distance]: ",
        platform_ceiling_distance_handle.error());
  }

  

  float lift_setpoint_position =
      platform_ceiling_distance +
      node_->get_parameter("platform_to_robot_base").as_double() +
      lift_current_position -
      node_->get_parameter("optimal_tcp_level").as_double() -
      node_->get_parameter("ptu_length").as_double() -
      node_->get_parameter("working_distance").as_double();

  RCLCPP_INFO(node_->get_logger(), "Calculated lift setpoint %f.",
              lift_setpoint_position);

  goal_msg.requested_target_lift_level = lift_setpoint_position;

  auto goal_handle_future = lift_move_action_client->async_send_goal(goal_msg);
  if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {

    RCLCPP_ERROR(node_->get_logger(), "Sending goal call failed.");
    return BT::NodeStatus::FAILURE;
  }

  rclcpp_action::ClientGoalHandle<
      bautiro_ros_interfaces::action::MoveLiftAbsolute>::SharedPtr goal_handle =
      goal_handle_future.get();
  if (!goal_handle) {

    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    return BT::NodeStatus::FAILURE;
  }

  auto result_future = lift_move_action_client->async_get_result(goal_handle);

  if (rclcpp::spin_until_future_complete(node_, result_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Failure in obtaining result.");
    return BT::NodeStatus::FAILURE;
  }

  rclcpp_action::ClientGoalHandle<
      bautiro_ros_interfaces::action::MoveLiftAbsolute>::WrappedResult
      wrapped_result = result_future.get();

  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED){
    // action is failed
    RCLCPP_ERROR(node_->get_logger(),
              "Lift driver: Result code: %d. Lift level: %f,",
              wrapped_result.result->response_code,
              wrapped_result.result->final_lift_level);
    return BT::NodeStatus::FAILURE;
  } 

  RCLCPP_INFO(node_->get_logger(),
              "Lift driver: Result code: %d. Lift level: %f,",
              wrapped_result.result->response_code,
              wrapped_result.result->final_lift_level);

  return BT::NodeStatus::SUCCESS;
}

} // namespace fpm_bt_tree_nodes

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {

  factory.registerNodeType<fpm_bt_tree_nodes::LiftMoveAbsolute>(
      "LiftMoveAbsolute");
  factory.registerNodeType<fpm_bt_tree_nodes::LiftMoveToWorkingHeight>(
      "LiftMoveToWorkingHeight");
}