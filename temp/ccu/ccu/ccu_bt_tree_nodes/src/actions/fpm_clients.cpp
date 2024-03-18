/* Copyright 2023 Robert Bosch GmbH and its subsidiaries

 All rights reserved, also regarding any disposal, exploitation, reproduction
 editing, distribution, as well as in the event of applications for industrial
 property rights. */


#include "ccu_bt_tree_nodes/actions/fpm_clients.hpp"

namespace ccu_bt_tree_nodes
{

//****************** FPMStartMission BT action node *****************************

FPMStartMission::FPMStartMission(
  const std::string& name,
  const BT::NodeConfiguration& config)
: BT::AsyncActionNode(name, config)
{
}

BT::NodeStatus FPMStartMission::tick()
{
  //Vars
  auto goal_msg = bautiro_ros_interfaces::action::StartFpmMainTask::Goal();
  rclcpp::Node::SharedPtr node_ = rclcpp::Node::make_shared("fpm_start_client");
  //ROS action/service client    
  auto action_client = rclcpp_action::create_client<bautiro_ros_interfaces::action::StartFpmMainTask>(node_, "/fpm_start");
  if (!action_client->wait_for_action_server(std::chrono::seconds(20))) 
  {
        return BT::NodeStatus::FAILURE;
  }
  //Read inputs and assign it to action goal message
  getInput<std::string>("behavior",goal_msg.bt_xml);
  getInput("marker_array", goal_msg.markers);
  getInput("drill_mask_array", goal_msg.drill_masks);
  
  //Assign action node inputs to action goal message 
  goal_msg.start = true;
  //Send goal message      
  auto goal_handle_future = action_client->async_send_goal(goal_msg);
  // Wait until the service response
  if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "send FPM Tree goal call failed");
        return BT::NodeStatus::FAILURE;
    }
    rclcpp_action::ClientGoalHandle<bautiro_ros_interfaces::action::StartFpmMainTask>::SharedPtr goal_handle 
      = goal_handle_future.get();
    if (!goal_handle) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "FPM Tree Goal was rejected by server");
        return BT::NodeStatus::FAILURE;
    }
    
  //Assign result to action result
  auto result_future = action_client->async_get_result(goal_handle);
    
  if (rclcpp::spin_until_future_complete(node_, result_future) !=
          rclcpp::FutureReturnCode::SUCCESS)
  {
      RCLCPP_ERROR(node_->get_logger(), "FPM Tree get result call failed " );
      return BT::NodeStatus::FAILURE;
  }
  rclcpp_action::ClientGoalHandle<bautiro_ros_interfaces::action::StartFpmMainTask>::WrappedResult wrapped_result
     = result_future.get();
  if(wrapped_result.result->message == "succeeded")
  {
    setOutput("drill_result", wrapped_result.result->drill_results);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "FPM task Complete!");        
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::SUCCESS;
}

}  // end namespace ccu_bt_tree_nodes

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{

  factory.registerNodeType<ccu_bt_tree_nodes::FPMStartMission>("StartFPM");
}