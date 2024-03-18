// Copyright (c) 2024 Robert Bosch GmbH and its subsidiaries
//
// All rights reserved, also regarding any disposal, exploitation, reproduction,
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#include "fpm_bt_tree_nodes/actions/hu_motion_manager_clients.hpp"

namespace fpm_bt_tree_nodes
{

ConfiguredPoseClient::ConfiguredPoseClient(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
}

inline BT::NodeStatus ConfiguredPoseClient::tick()
{
  
      rclcpp::Node::SharedPtr node_;
      node_ = rclcpp::Node::make_shared("start_action");
      auto action_client = rclcpp_action::create_client<bautiro_ros_interfaces::action::SetHandlingUnitConfiguredPose>(node_, "/fpm_set_hu_configured_pose");
      if (!action_client->wait_for_action_server(std::chrono::seconds(20))) {
            return BT::NodeStatus::FAILURE;
        }
      auto goal_msg = bautiro_ros_interfaces::action::SetHandlingUnitConfiguredPose::Goal();
      getInput<int32_t>("handling_unit_configured_pose",goal_msg.handling_unit_configured_pose);
      auto goal_handle_future = action_client->async_send_goal(goal_msg);

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending goal %d", goal_msg.handling_unit_configured_pose);
      if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
                rclcpp::FutureReturnCode::SUCCESS)
        {
            
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "send goal call failed");
            return BT::NodeStatus::FAILURE;
        }
        rclcpp_action::ClientGoalHandle<bautiro_ros_interfaces::action::SetHandlingUnitConfiguredPose>::SharedPtr goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal was rejected by server");
            return BT::NodeStatus::FAILURE;
        }
        
        
        auto result_future = action_client->async_get_result(goal_handle);
        
        if (rclcpp::spin_until_future_complete(node_, result_future) !=
                rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "get result call failed " );
            return BT::NodeStatus::FAILURE;
        }
        rclcpp_action::ClientGoalHandle<bautiro_ros_interfaces::action::SetHandlingUnitConfiguredPose>::WrappedResult wrapped_result = result_future.get();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "result received");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "the response code is %d" ,wrapped_result.result->response_code);
        
        return BT::NodeStatus::SUCCESS;
}

}  // namespace fpm_bt_tree_nodes


namespace fpm_bt_tree_nodes
{
MoveAbsoluteClient::MoveAbsoluteClient(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  
}

inline BT::NodeStatus MoveAbsoluteClient::tick()
{
      rclcpp::Node::SharedPtr node_;
      node_ = rclcpp::Node::make_shared("start_action");
      auto action_client = rclcpp_action::create_client<bautiro_ros_interfaces::action::MoveHandlingUnitAbsolute>(node_, "/fpm_set_move_hu_absolute");
      if (!action_client->wait_for_action_server(std::chrono::seconds(20))) {
            return BT::NodeStatus::FAILURE;
        }
      auto goal_msg = bautiro_ros_interfaces::action::MoveHandlingUnitAbsolute::Goal();
    //   Pose2D goal;
      geometry_msgs::msg::PoseStamped goal;
      getInput<geometry_msgs::msg::PoseStamped>("absolute_target_position",goal);
      goal_msg.absolute_target_position.header.frame_id = "base_link_inertia";
      goal_msg.absolute_target_position.pose.position.x = goal.pose.position.x;
      goal_msg.absolute_target_position.pose.position.y = goal.pose.position.y;
      goal_msg.absolute_target_position.pose.position.z = goal.pose.position.z;
      goal_msg.absolute_target_position.pose.orientation.x = goal.pose.orientation.x;
      goal_msg.absolute_target_position.pose.orientation.y = goal.pose.orientation.y;
      goal_msg.absolute_target_position.pose.orientation.z = goal.pose.orientation.z;
      goal_msg.absolute_target_position.pose.orientation.w = goal.pose.orientation.w;
      auto goal_handle_future = action_client->async_send_goal(goal_msg);
      if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
                rclcpp::FutureReturnCode::SUCCESS)
        {
            
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "send goal call failed");
            return BT::NodeStatus::FAILURE;
        }
        rclcpp_action::ClientGoalHandle<bautiro_ros_interfaces::action::MoveHandlingUnitAbsolute>::SharedPtr goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal was rejected by server");
            return BT::NodeStatus::FAILURE;
        }
        
        
        auto result_future = action_client->async_get_result(goal_handle);
        
        if (rclcpp::spin_until_future_complete(node_, result_future) !=
                rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "get result call failed " );
            return BT::NodeStatus::FAILURE;
        }
        rclcpp_action::ClientGoalHandle<bautiro_ros_interfaces::action::MoveHandlingUnitAbsolute>::WrappedResult wrapped_result = result_future.get();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "result received");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "the response code is %d" ,wrapped_result.result->response_code);
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "the final lift level is %f" ,wrapped_result.result->final_lift_level);

        
        return BT::NodeStatus::SUCCESS;
}

}  // namespace fpm_bt_tree_nodes


namespace fpm_bt_tree_nodes
{
MoveRelativeClient::MoveRelativeClient(
  const std::string & name,
  const BT::NodeConfiguration & conf)
:  BT::ActionNodeBase(name, conf)
{
  
}

inline BT::NodeStatus MoveRelativeClient::tick()
{
    rclcpp::Node::SharedPtr node_;
    node_ = rclcpp::Node::make_shared("start_action");
    auto action_client = rclcpp_action::create_client<bautiro_ros_interfaces::action::MoveHandlingUnitRelativToWorkplane>(node_, "/fpm_set_move_relative");
    if (!action_client->wait_for_action_server(std::chrono::seconds(20))) {
          return BT::NodeStatus::FAILURE;
      }
    auto goal_msg = bautiro_ros_interfaces::action::MoveHandlingUnitRelativToWorkplane::Goal();
    // Pose2D goal;

    float x;
    float y;
    auto x_handle = getInput<float>("x",x);
    if (!x_handle.has_value()) {
      throw BT::RuntimeError("Missing required input [x]: ",
                             x_handle.error());
    }
    auto y_handle = getInput<float>("y", y);
    if (!y_handle.has_value()) {
      throw BT::RuntimeError("Missing required input [y]: ",
                             y_handle.error());
    }
    goal_msg.relative_target_position={x,y};
    auto goal_handle_future = action_client->async_send_goal(goal_msg);
    if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
              rclcpp::FutureReturnCode::SUCCESS)
    {
        
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "send goal call failed");
        return BT::NodeStatus::FAILURE;
    }
    rclcpp_action::ClientGoalHandle<bautiro_ros_interfaces::action::MoveHandlingUnitRelativToWorkplane>::SharedPtr goal_handle = goal_handle_future.get();
    if (!goal_handle) {
        
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal was rejected by server");
        return BT::NodeStatus::FAILURE;
    }
    
    auto result_future = action_client->async_get_result(goal_handle);
    
    if (rclcpp::spin_until_future_complete(node_, result_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "get result call failed " );
        return BT::NodeStatus::FAILURE;
    }
    rclcpp_action::ClientGoalHandle<bautiro_ros_interfaces::action::MoveHandlingUnitRelativToWorkplane>::WrappedResult wrapped_result = result_future.get();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "result received");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "the response code is %d" ,wrapped_result.result->response_code);
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "the final lift level is %f" ,wrapped_result.result->final_lift_level);
    
    return BT::NodeStatus::SUCCESS;
}

}  // namespace fpm_bt_tree_nodes

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<fpm_bt_tree_nodes::ConfiguredPoseClient>("ConfiguredPose");
    // factory.registerNodeType<fpm_bt_tree_nodes::MoveLiftAbsoluteClient>("MoveLiftAbsolute");
    factory.registerNodeType<fpm_bt_tree_nodes::MoveAbsoluteClient>("MoveAbsolute");
    factory.registerNodeType<fpm_bt_tree_nodes::MoveRelativeClient>("MoveRelative");
}