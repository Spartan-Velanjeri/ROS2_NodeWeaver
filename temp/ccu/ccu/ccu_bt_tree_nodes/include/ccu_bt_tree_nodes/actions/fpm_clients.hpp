/* Copyright 2023 Robert Bosch GmbH and its subsidiaries

 All rights reserved, also regarding any disposal, exploitation, reproduction
 editing, distribution, as well as in the event of applications for industrial
 property rights. */

//common

#ifndef BEHAVIOR_TREE__PLUGINS__ACTION__FPM_START_MISSION
#define BEHAVIOR_TREE__PLUGINS__ACTION__FPM_START_MISSION

#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <functional>
#include <behaviortree_cpp_v3/action_node.h>

//ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


//BAUTIRO
#include "bautiro_ros_interfaces/action/start_fpm_main_task.hpp"
#include "bautiro_ros_interfaces/msg/drill_hole_feedback.hpp"

namespace ccu_bt_tree_nodes
{

class FPMStartMission: public BT::AsyncActionNode
{
public:
            
    FPMStartMission(
        const std::string& name,
        const BT::NodeConfiguration& config);
        
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts()
    {
        return
        {
        BT::InputPort<std::string>("behavior"),
        BT::InputPort<std::vector<bautiro_ros_interfaces::msg::DrillMask>>("drill_mask_array"),
        BT::InputPort<std::vector<bautiro_ros_interfaces::msg::Marker>>("marker_array"),
        BT::OutputPort<std::vector<bautiro_ros_interfaces::msg::DrillHoleFeedback>>("drill_result")
        };
    } 
};

}   


#endif  // BEHAVIOR_TREE__PLUGINS__ACTION__FPM_START_MISSION
