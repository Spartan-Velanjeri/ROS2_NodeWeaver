//  Copyright 2024 Robert Bosch GmbH and its subsidiaries
//
//  All rights reserved, also regarding any disposal, exploitation, reproduction,
//  editing, distribution, as well as in the event of applications for industrial
//  property rights.
#ifndef RPM_BT_TREE_NODES__ACTIONS__TRANSFORM_TARGET_POSE_ASYNC_HPP_
#define RPM_BT_TREE_NODES__ACTIONS__TRANSFORM_TARGET_POSE_ASYNC_HPP_



//  Actions, services, messages

#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <rclcpp/parameter.hpp>

#include <string>
#include <memory>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <vector>
#include <functional>
#include <thread>

//  ROS includes
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "visualization_msgs/msg/marker.hpp"



namespace rpm_behavior_tree
{


/**
 * @brief A rpm_behavior_tree::BtActionNode class that the transforms the target pose to ur_robot's frame
 */
class Transform_target_async : public BT::StatefulActionNode
    {
     public:
        Transform_target_async(const std::string& name, const BT::NodeConfiguration& config)
            : StatefulActionNode(name, config)
        {
        }
        static BT::PortsList providedPorts()
        {
          return
            {
            BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("offset_target_pose")
            };
        }

        BT::NodeStatus onStart() override;

        BT::NodeStatus onRunning() override;

        void onHalted() override;
    };

class Transform_calculation : public rclcpp::Node
    {
        public:
            Transform_calculation();
            void transform_point();
    };
}  // namespace rpm_behavior_tree

#endif  // RPM_BT_TREE_NODES__ACTIONS__TRANSFORM_TARGET_POSE_ASYNC_HPP_
