/* Copyright 2023 Robert Bosch GmbH and its subsidiaries

 All rights reserved, also regarding any disposal, exploitation, reproduction
 editing, distribution, as well as in the event of applications for industrial
 property rights. */


#ifndef BEHAVIOR_TREE__PLUGINS__ACTION__START_RPM_TASK
#define BEHAVIOR_TREE__PLUGINS__ACTION__START_RPM_TASK

 //common
#include <behaviortree_cpp_v3/action_node.h>

//ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

//BAUTIRO
#include "bautiro_ros_interfaces/action/start_rpm_main_task.hpp"
#include "bautiro_ros_interfaces/action/move_base_link.hpp"
#include "bautiro_ros_interfaces/action/move_ur_base.hpp"
#include "bautiro_ros_interfaces/msg/navigation_result.hpp"


namespace ccu_bt_tree_nodes
{

    class StartRPMTask: public BT::AsyncActionNode
    {
    public:

        bool wait_for_cluster = false;
        
        StartRPMTask(
            const std::string& name,
            const BT::NodeConfiguration& config);
            

        static BT::PortsList providedPorts()
        {
            {
              return {
                BT::InputPort<std::string>("behavior"), 
                BT::InputPort<geometry_msgs::msg::Pose>("navigation_goal"),
                BT::InputPort<std::string>("navigation_goal_id"), 
                BT::OutputPort<bautiro_ros_interfaces::msg::NavigationResult>("navigation_result") };
            };
        }

        BT::NodeStatus tick() override;
    

    };

}   

#endif  // BEHAVIOR_TREE__PLUGINS__ACTION__START_MISSION