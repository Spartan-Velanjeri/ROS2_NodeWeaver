// Copyright (c) 2024 Robert Bosch GmbH and its subsidiaries
//
// All rights reserved, also regarding any disposal, exploitation, reproduction,
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#ifndef BEHAVIOR_TREE__PLUGINS__ACTION__TF_POSE_STAMPED_NODE_HPP_
#define BEHAVIOR_TREE__PLUGINS__ACTION__TF_POSE_STAMPED_NODE_HPP_

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
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/buffer.h"
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
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
#include "visualization_msgs/msg/marker.hpp"
namespace fpm_bt_tree_nodes
{


class TransformDrillHole : public BT::ActionNodeBase
{
public:
  
  TransformDrillHole(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

 
  BT::NodeStatus tick() override;

  void halt() override {}
  static BT::PortsList providedPorts()
  {
    return
      {
       BT::InputPort<geometry_msgs::msg::PoseStamped>("in_pose"),
       BT::InputPort<geometry_msgs::msg::PoseStamped>("in_target_frame_id"),
       BT::OutputPort<geometry_msgs::msg::PoseStamped>("out_pose")       
      };
  }
  geometry_msgs::msg::PoseStamped in_pose, out_pose;
  std::string in_target_frame_id;
  geometry_msgs::msg::TransformStamped t1 = geometry_msgs::msg::TransformStamped();
  geometry_msgs::msg::TransformStamped t_UrBaseNew_BaseLink;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
};

}  // namespace fpm_bt_tree_nodes

 #endif  // BEHAVIOR_TREE__PLUGINS__ACTION__TF_POSE_STAMPED_NODE_HPP_