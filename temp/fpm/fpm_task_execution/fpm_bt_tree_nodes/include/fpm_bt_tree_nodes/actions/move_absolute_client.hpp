// Copyright (c) 2024 Robert Bosch GmbH and its subsidiaries
//
// All rights reserved, also regarding any disposal, exploitation, reproduction,
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#ifndef BEHAVIOR_TREE__PLUGINS__ACTION__SECOND_NODE_HPP_
#define BEHAVIOR_TREE__PLUGINS__ACTION__SECOND_NODE_HPP_

#include <string>
#include <string>
#include <memory>
#include <algorithm>
#include <chrono>
#include <iostream>
#include<vector>
#include <functional>
#include <memory>
#include <thread>
//Actions, services, messages
#include "bautiro_ros_interfaces/action/move_handling_unit_absolute.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

//ROS includes
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/parameter.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
using namespace std;
using namespace std::chrono;
#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace fpm_bt_tree_nodes
{


class MoveAbsoluteClient : public BT::ActionNodeBase
{
public:
  
  MoveAbsoluteClient(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

 
  BT::NodeStatus tick() override;

  void halt() override {}
  static BT::PortsList providedPorts()
  {
    return
      {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("absolute_target_position")
      };
  }
  
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__BACK_UP_ACTION_HPP_