// Copyright (c) 2024 Robert Bosch GmbH and its subsidiaries
//
// All rights reserved, also regarding any disposal, exploitation, reproduction,
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#ifndef BEHAVIOR_TREE__PLUGINS__ACTION__HU_NODE_HPP_
#define BEHAVIOR_TREE__PLUGINS__ACTION__HU_NODE_HPP_

#include <functional>
#include <memory>
#include <string>
#include <thread>

// Actions, services, messages
#include "bautiro_ros_interfaces/action/start_hu_main_task.hpp"
#include "bautiro_ros_interfaces/action/move_handling_unit_relativ_to_workplane.hpp"
#include <action_msgs/msg/goal_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>

// ROS includes
#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/int32.hpp"
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
namespace fpm_bt_tree_nodes {

/**
 * @brief Action node for hu skills. Inputs:
 *        skill_name (string)
 *
 */
class HUSkillExecute : public BT::ActionNodeBase {
public:
  HUSkillExecute(const std::string &xml_tag_name,
                      const BT::NodeConfiguration &conf);

  BT::NodeStatus tick() override;
  void halt() override {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std_msgs::msg::String>("skill_name")};
  }
};

} // namespace fpm_bt_tree_nodes

#endif // BEHAVIOR_TREE__PLUGINS__ACTION__HU_NODE_HPP_
