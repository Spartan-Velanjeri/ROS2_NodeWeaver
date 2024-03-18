// Copyright (c) 2024 Robert Bosch GmbH and its subsidiaries
//
// All rights reserved, also regarding any disposal, exploitation, reproduction,
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#ifndef BEHAVIOR_TREE__PLUGINS__ACTION__FIRST_NODE_HPP_
#define BEHAVIOR_TREE__PLUGINS__ACTION__FIRST_NODE_HPP_

#include <string>
#include <string>
#include <memory>
#include "behaviortree_cpp_v3/action_node.h"
#include <rclcpp/rclcpp.hpp>
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "std_msgs/msg/int32.hpp"
#include <functional>
#include <memory>
#include <thread>
//Actions, services, messages
#include "bautiro_ros_interfaces/action/set_handling_unit_configured_pose.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

//ROS includes
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/parameter.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
namespace fpm_bt_tree_nodes
{


class ConfiguredPoseClient : public BT::ActionNodeBase
{
public:
  
  ConfiguredPoseClient(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;
  void halt() override {}
  static BT::PortsList providedPorts()
  {
    return
      {
        BT::InputPort<int32_t>("handling_unit_configured_pose")
      };
  }
  int initial, total;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__BACK_UP_ACTION_HPP_