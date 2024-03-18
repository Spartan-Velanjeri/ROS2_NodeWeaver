// Copyright (c) 2024 Robert Bosch GmbH and its subsidiaries
//
// All rights reserved, also regarding any disposal, exploitation, reproduction,
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#ifndef BEHAVIOR_TREE__PLUGINS__ACTION__FOURTH_NODE_HPP_SIM_
#define BEHAVIOR_TREE__PLUGINS__ACTION__FOURTH_NODE_HPP_SIM_
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <behaviortree_cpp_v3/action_node.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <yaml-cpp/yaml.h>
#include <algorithm>  // std::min
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <atomic>
#include <geometry_msgs/msg/pose.hpp>
#include <string>
#include <vector>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "std_msgs/msg/int32.hpp"
#include "bautiro_ros_interfaces/srv/get_drill_holes_cluster.hpp"
#include "bautiro_ros_interfaces/msg/drill_mask.hpp"
namespace fpm_bt_tree_nodes
{


class GetPoi : public BT::ActionNodeBase
{
public:
  
  GetPoi(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;
  void halt() override {}
  static BT::PortsList providedPorts()
  {
    return
      {
        BT::InputPort<int32_t>("total"),
        BT::InputPort<std::vector<bautiro_ros_interfaces::msg::DrillMask>>("point_list"), 
        BT::InputPort<int32_t>("point_index"), BT::OutputPort<geometry_msgs::msg::Pose>("drill_point"), BT::OutputPort<geometry_msgs::msg::Pose>("end_point")
      };
  }
  int initial, total;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__BACK_UP_ACTION_HPP_SIM_