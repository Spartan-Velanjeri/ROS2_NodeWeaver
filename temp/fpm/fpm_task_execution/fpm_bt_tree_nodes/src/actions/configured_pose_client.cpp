// Copyright (c) 2024 Robert Bosch GmbH and its subsidiaries
//
// All rights reserved, also regarding any disposal, exploitation, reproduction,
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#include "fpm_bt_tree_nodes/actions/configured_pose_client.hpp"

namespace fpm_bt_tree_nodes {

ConfiguredPoseClient::ConfiguredPoseClient(const std::string &name,
                                           const BT::NodeConfiguration &conf)
    : BT::ActionNodeBase(name, conf) {}

inline BT::NodeStatus ConfiguredPoseClient::tick() {

  //get requested pose
  auto goal_msg =
      bautiro_ros_interfaces::action::SetHandlingUnitConfiguredPose::Goal();
  auto getInputResult = getInput<int32_t>("handling_unit_configured_pose",
                    goal_msg.handling_unit_configured_pose);

  if (!getInputResult .has_value()) {
    throw BT::RuntimeError("Missing required input [handling_unit_configured_pose]: ",
                           getInputResult.error());
  }               

  if (0 > goal_msg.handling_unit_configured_pose ||  goal_msg.handling_unit_configured_pose>5){
    throw BT::RuntimeError("Undefined configured pose. Defined poses are in range 0-5:",
                           getInputResult.error());
  } 

  rclcpp::Node::SharedPtr node_;
  node_ = rclcpp::Node::make_shared("configured_pose_client");
  node_->declare_parameter("client_wait_time", 2);

  auto action_client = rclcpp_action::create_client<
      bautiro_ros_interfaces::action::SetHandlingUnitConfiguredPose>(
      node_, "/fpm_set_hu_configured_pose");

  if (!action_client->wait_for_action_server(std::chrono::seconds(
          node_->get_parameter("client_wait_time").as_int()))) {
    RCLCPP_ERROR(node_->get_logger(), "Robot service not available.");
    return BT::NodeStatus::FAILURE;
  }
  
  auto goal_handle_future = action_client->async_send_goal(goal_msg);
  if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {

    RCLCPP_INFO(node_->get_logger(), "send goal call failed");
    return BT::NodeStatus::FAILURE;
  }
  rclcpp_action::ClientGoalHandle<
      bautiro_ros_interfaces::action::SetHandlingUnitConfiguredPose>::SharedPtr
      goal_handle = goal_handle_future.get();
  if (!goal_handle) {

    RCLCPP_INFO(node_->get_logger(), "Goal was rejected by server");
    return BT::NodeStatus::FAILURE;
  }

  auto result_future = action_client->async_get_result(goal_handle);

  if (rclcpp::spin_until_future_complete(node_, result_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "get result call failed ");
    return BT::NodeStatus::FAILURE;
  }
  rclcpp_action::ClientGoalHandle<
      bautiro_ros_interfaces::action::SetHandlingUnitConfiguredPose>::
      WrappedResult wrapped_result = result_future.get();

  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED){
    // action is failed
    RCLCPP_ERROR(node_->get_logger(),
              "HU configured pose: Error: %d",wrapped_result.result->response_code);
    return BT::NodeStatus::FAILURE;
  } 

  RCLCPP_INFO(node_->get_logger(),
              "HU configured pose: Success. Response code: %d",wrapped_result.result->response_code);

  return BT::NodeStatus::SUCCESS;
}

} // namespace fpm_bt_tree_nodes

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {

  factory.registerNodeType<fpm_bt_tree_nodes::ConfiguredPoseClient>(
      "ConfiguredPose");
}