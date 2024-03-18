#ifndef BEHAVIOR_TREE__PLUGINS__ACTION__CONFIGURED_POSE_NODE_HPP_
#define BEHAVIOR_TREE__PLUGINS__ACTION__CONFIGURED_POSE_NODE_HPP_


#include <string>
#include <memory>
#include <functional>
#include <thread>
#include "behaviortree_cpp_v3/action_node.h"

//Actions, services, messages
#include "std_msgs/msg/int32.hpp"
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
#include "nav2_behavior_tree/bt_action_node.hpp"

//Bautiro
#include "bautiro_ros_interfaces/action/set_handling_unit_configured_pose.hpp"

namespace hu_bt_tree_nodes
{


class ConfiguredPoseClient : public BT::ActionNodeBase
{
public:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<bautiro_ros_interfaces::action::SetHandlingUnitConfiguredPose>::SharedPtr action_client_;
  rclcpp_action::Client<bautiro_ros_interfaces::action::SetHandlingUnitConfiguredPose>::Goal goal_msg_ 
    = bautiro_ros_interfaces::action::SetHandlingUnitConfiguredPose::Goal();
  rclcpp_action::ClientGoalHandle<bautiro_ros_interfaces::action::SetHandlingUnitConfiguredPose>::SharedPtr goal_handle_;
  rclcpp_action::ClientGoalHandle<bautiro_ros_interfaces::action::SetHandlingUnitConfiguredPose>::WrappedResult wrapped_result_;
  std::string node_name_ = "bt_node_configured_pose_client";
  std::string server_name_ = "/fpm_set_hu_configured_pose";


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

}  // namespace hu_bt_tree_nodes

#endif  // BEHAVIOR_TREE__PLUGINS__ACTION__CONFIGURED_POSE_NODE_HPP_


#ifndef BEHAVIOR_TREE__PLUGINS__ACTION__MOVE_ABSOLUT_NODE_HPP_
#define BEHAVIOR_TREE__PLUGINS__ACTION__MOVE_ABSOLUT_NODE_HPP_

#include <algorithm>
#include <chrono>
#include <iostream>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

//Actions, services, messages
#include <geometry_msgs/msg/pose_stamped.hpp>

//ROS includes
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/parameter.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_behavior_tree/bt_action_node.hpp"

//Bautiro
#include "bautiro_ros_interfaces/action/move_handling_unit_absolute.hpp"

using namespace std;
using namespace std::chrono;


namespace hu_bt_tree_nodes
{
/**
 * @brief Action node for absolute movement of the handling unit. Inputs:
 *        absolute_target_position (geometry_msgs::msg::PoseStamped)
 *
 */
class MoveAbsoluteClient : public BT::ActionNodeBase
{
public:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<bautiro_ros_interfaces::action::MoveHandlingUnitAbsolute>::SharedPtr action_client_;
  rclcpp_action::Client<bautiro_ros_interfaces::action::MoveHandlingUnitAbsolute>::Goal goal_msg_ 
    = bautiro_ros_interfaces::action::MoveHandlingUnitAbsolute::Goal();
  rclcpp_action::ClientGoalHandle<bautiro_ros_interfaces::action::MoveHandlingUnitAbsolute>::SharedPtr goal_handle_;
  rclcpp_action::ClientGoalHandle<bautiro_ros_interfaces::action::MoveHandlingUnitAbsolute>::WrappedResult wrapped_result_;
  string node_name_ = "bt_node_move-absolut_client";
  string server_name_ = "/fpm_set_move_hu_absolute";

  geometry_msgs::msg::PoseStamped goal_pose_;

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

}  // namespace hu_bt_tree_nodes

#endif  // BEHAVIOR_TREE__PLUGINS__ACTION__MOVE_ABSOLUT_NODE_HPP_

#ifndef BEHAVIOR_TREE__PLUGINS__ACTION__FOURTH_NODE_HPP_
#define BEHAVIOR_TREE__PLUGINS__ACTION__FOURTH_NODE_HPP_

#include <string>
#include <string>
#include <memory>
#include <functional>
#include <memory>
#include <thread>

//Actions, services, messages
#include "std_msgs/msg/int32.hpp"
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

//ROS includes
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/parameter.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_behavior_tree/bt_action_node.hpp"

//Bautiro
#include "bautiro_ros_interfaces/action/move_handling_unit_relativ_to_workplane.hpp"
namespace hu_bt_tree_nodes
{


class MoveRelativeClient : public BT::ActionNodeBase
{
public:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<bautiro_ros_interfaces::action::MoveHandlingUnitRelativToWorkplane>::SharedPtr action_client_;
  rclcpp_action::Client<bautiro_ros_interfaces::action::MoveHandlingUnitRelativToWorkplane>::Goal goal_msg_ 
    = bautiro_ros_interfaces::action::MoveHandlingUnitRelativToWorkplane::Goal();
  rclcpp_action::ClientGoalHandle<bautiro_ros_interfaces::action::MoveHandlingUnitRelativToWorkplane>::SharedPtr goal_handle_;
  rclcpp_action::ClientGoalHandle<bautiro_ros_interfaces::action::MoveHandlingUnitRelativToWorkplane>::WrappedResult wrapped_result_;
  string node_name_ = "bt_node_move_relaitve_client";
  string server_name_ = "/fpm_set_move_relative";  float x;
  float y;


  MoveRelativeClient(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;
  void halt() override {}
  static BT::PortsList providedPorts()
  {
    return
      {
      BT::InputPort<float>("x"),
      BT::InputPort<float>("y")
      };
  }
  
};

}  // namespace hu_bt_tree_nodes

#endif  // BEHAVIOR_TREE__PLUGINS__ACTION__FOURTH_NODE_HPP_