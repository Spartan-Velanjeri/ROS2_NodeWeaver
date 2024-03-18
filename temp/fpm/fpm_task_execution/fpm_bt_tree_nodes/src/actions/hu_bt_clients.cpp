// Copyright (c) 2024 Robert Bosch GmbH and its subsidiaries
//
// All rights reserved, also regarding any disposal, exploitation, reproduction,
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#include "fpm_bt_tree_nodes/actions/hu_bt_clients.hpp"

namespace fpm_bt_tree_nodes {

//------------------------HUSkillExecute-------------------------------
HUSkillExecute::HUSkillExecute(const std::string &name,
                               const BT::NodeConfiguration &conf)
    : BT::ActionNodeBase(name, conf) {}

inline BT::NodeStatus HUSkillExecute::tick() {

  rclcpp::Node::SharedPtr node_;
  node_ = rclcpp::Node::make_shared("hu_skill_execute");

  node_->declare_parameter("client_wait_time", 2);

  // Create action client for the hu skill service
  auto action_client = rclcpp_action::create_client<
      bautiro_ros_interfaces::action::StartHuMainTask>(node_,
                                                       "/hu_start");

  if (!action_client->wait_for_action_server(std::chrono::seconds(
          node_->get_parameter("client_wait_time").as_int()))) {
    RCLCPP_ERROR(node_->get_logger(), "HU main skill server not available.");
    return BT::NodeStatus::FAILURE;
  }

  // send skill name to be executed
  auto goal_msg = bautiro_ros_interfaces::action::StartHuMainTask::Goal();
  // RCLCPP_INFO(node_->get_logger(), "Message after init: %s",
  //             goal_msg.bt_xml.c_str());

  auto bt_xml_handle = getInput<std::string>("skill_name", goal_msg.bt_xml);

  if (!bt_xml_handle.has_value()) {
    throw BT::RuntimeError("Missing required input [bt_xml]: ",
                           bt_xml_handle.error());
  }
  goal_msg.start = true;
  auto goal_handle_future = action_client->async_send_goal(goal_msg);
  if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {

    RCLCPP_ERROR(node_->get_logger(), "Sending goal call failed.");
    return BT::NodeStatus::FAILURE;
  }

  rclcpp_action::ClientGoalHandle<
      bautiro_ros_interfaces::action::StartHuMainTask>::SharedPtr goal_handle =
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
      bautiro_ros_interfaces::action::StartHuMainTask>::WrappedResult
      wrapped_result = result_future.get();

  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    // action is failed
    // RCLCPP_ERROR(node_->get_logger(), "Hu skill server: Result message: %s.",
    //              wrapped_result.result->message.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // RCLCPP_INFO(node_->get_logger(), "Hu skill server: Result message: %s.",
  //             wrapped_result.result->message.c_str());

  return BT::NodeStatus::SUCCESS;
}

} // namespace fpm_bt_tree_nodes


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<fpm_bt_tree_nodes::HUSkillExecute>("HUSkillExecute");
}
