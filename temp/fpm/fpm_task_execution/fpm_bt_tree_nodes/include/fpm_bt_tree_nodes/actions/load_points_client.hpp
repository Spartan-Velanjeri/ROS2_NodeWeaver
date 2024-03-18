// Copyright (c) 2024 Robert Bosch GmbH and its subsidiaries
//
// All rights reserved, also regarding any disposal, exploitation, reproduction,
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#ifndef BEHAVIOR_TREE__PLUGINS__ACTION__SIXTH_NODE_HPP_SIM_
#define BEHAVIOR_TREE__PLUGINS__ACTION__SIXTH_NODE_HPP_SIM_

#include <string>
#include <rclcpp/rclcpp.hpp>
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <rclcpp/rclcpp.hpp>
#include "bautiro_ros_interfaces/srv/get_drill_holes_cluster.hpp"
#include "bautiro_ros_interfaces/msg/drill_mask.hpp"
#include <chrono>
#include <behaviortree_cpp_v3/condition_node.h>
#include <unistd.h>

#include <string>
#include <memory>
namespace fpm_bt_tree_nodes
{


class LoadPointsClient : public BT::AsyncActionNode
{
public:
  
  LoadPointsClient(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;
  void halt() override {}
  static BT::PortsList providedPorts()
  {
    return
      {
        BT::OutputPort<std::vector<bautiro_ros_interfaces::msg::DrillMask>>("pointslist") , 
        BT::OutputPort<int>("totalpoints")
      };
  }
  std::vector<bautiro_ros_interfaces::msg::DrillMask> array;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__BACK_UP_ACTION_HPP_SIM_