// Copyright (c) 2024 Robert Bosch GmbH and its subsidiaries
//
// All rights reserved, also regarding any disposal, exploitation, reproduction,
// editing, distribution, as well as in the event of applications for industrial
// property rights.


#include "fpm_bt_tree_nodes/actions/move_absolute_client_sim.hpp"


namespace fpm_bt_tree_nodes
{
MoveAbsoluteClientSim::MoveAbsoluteClientSim(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  
}

inline BT::NodeStatus MoveAbsoluteClientSim::tick()
{
      rclcpp::Node::SharedPtr node_;
      node_ = rclcpp::Node::make_shared("start_action");
      auto action_client = rclcpp_action::create_client<bautiro_ros_interfaces::action::MoveHandlingUnitAbsolute>(node_, "/fpm_set_move_hu_absolute");
      if (!action_client->wait_for_action_server(std::chrono::seconds(20))) {
            return BT::NodeStatus::FAILURE;
        }
      auto goal_msg = bautiro_ros_interfaces::action::MoveHandlingUnitAbsolute::Goal();
      //Pose2D goal;
      geometry_msgs::msg::Pose goal;
      getInput<geometry_msgs::msg::Pose>("absolute_target_position",goal);
      goal_msg.absolute_target_position.header.frame_id = "base_link_inertia";
      goal_msg.absolute_target_position.pose.position.x = goal.position.x;
      goal_msg.absolute_target_position.pose.position.y = goal.position.y;
      goal_msg.absolute_target_position.pose.position.z = goal.position.z;
      goal_msg.absolute_target_position.pose.orientation.x = goal.orientation.x;
      goal_msg.absolute_target_position.pose.orientation.y = goal.orientation.y;
      goal_msg.absolute_target_position.pose.orientation.z = goal.orientation.z;
      goal_msg.absolute_target_position.pose.orientation.w = goal.orientation.w;
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

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{

  factory.registerNodeType<fpm_bt_tree_nodes::MoveAbsoluteClientSim>("moveabsolute_sim");
}