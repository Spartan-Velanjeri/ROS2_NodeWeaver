/* Copyright 2023 Robert Bosch GmbH and its subsidiaries

 All rights reserved, also regarding any disposal, exploitation, reproduction
 editing, distribution, as well as in the event of applications for industrial
 property rights. */

#include "ccu_bt_tree_nodes/actions/rpm_clients.hpp"
  

namespace ccu_bt_tree_nodes
{
//****************** StartRPMTask BT action node *****************************
    StartRPMTask::StartRPMTask(
        const std::string& name,
        const BT::NodeConfiguration& config)
    : BT::AsyncActionNode(name, config)
    {
    }

    BT::NodeStatus StartRPMTask::tick()
    {   
        //Vars
        auto goal_msg = bautiro_ros_interfaces::action::StartRpmMainTask::Goal();

        //ROS action/service client
        rclcpp::Node::SharedPtr node_=rclcpp::Node::make_shared("start_rpm_main_task");
        auto action_client = rclcpp_action::create_client<bautiro_ros_interfaces::action::StartRpmMainTask>(node_, "/rpm_start");
        
        if (!action_client->wait_for_action_server(std::chrono::seconds(20))) {
            return BT::NodeStatus::FAILURE;
        }

        //Read inputs and assign it to the action goal message
        getInput<std::string>("behavior",goal_msg.bt_xml);
        getInput<geometry_msgs::msg::Pose>("navigation_goal",goal_msg.des_cluster_pose.pks_pose);
        getInput<std::string>("navigation_goal_id",goal_msg.des_cluster_pose.cluster_id);

        //Assign action node inputs to action goal message 
        goal_msg.start = true; //Always start the behavior tree

        //Send goal message
        auto goal_handle_future = action_client->async_send_goal(goal_msg);

        // Wait until the service response
        if (rclcpp::spin_until_future_complete(node_, goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Send RPM Tree goal call failed");
            return BT::NodeStatus::FAILURE;
        }
        rclcpp_action::ClientGoalHandle<bautiro_ros_interfaces::action::StartRpmMainTask>::SharedPtr goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RPM Tree Goal was rejected by server");
            return BT::NodeStatus::FAILURE;
        }


        //Assign result to action result
        auto result_future = action_client->async_get_result(goal_handle);
        
        if (rclcpp::spin_until_future_complete(node_, result_future) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "RPM Tree goal call failed" );
            return BT::NodeStatus::FAILURE;
        }
        else
        {
            rclcpp_action::ClientGoalHandle<bautiro_ros_interfaces::action::StartRpmMainTask>::WrappedResult wrapped_result = result_future.get();
            if(wrapped_result.result->message == "succeeded")
            {
                setOutput("navigation_result", wrapped_result.result->nav_result);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pose Reached!");  
                return BT::NodeStatus::SUCCESS;
            }
        }
        
        return BT::NodeStatus::SUCCESS;
    }


}
  
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ccu_bt_tree_nodes::StartRPMTask>("StartRPM");
  
}


