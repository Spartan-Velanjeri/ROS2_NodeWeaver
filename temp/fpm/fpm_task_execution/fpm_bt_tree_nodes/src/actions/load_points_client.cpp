// Copyright (c) 2024 Robert Bosch GmbH and its subsidiaries
//
// All rights reserved, also regarding any disposal, exploitation, reproduction,
// editing, distribution, as well as in the event of applications for industrial
// property rights.


#include "fpm_bt_tree_nodes/actions/load_points_client.hpp"

 
using namespace std::chrono_literals; 
geometry_msgs::msg::PoseStamped p;
namespace fpm_bt_tree_nodes
{

LoadPointsClient::LoadPointsClient(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::AsyncActionNode(name, conf)
{
  
}

inline BT::NodeStatus LoadPointsClient::tick()
{
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("load_points_client");
  getInput<std::vector<bautiro_ros_interfaces::msg::DrillMask>>("pointslist" ,array);
  setOutput("totalpoints", array.size());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Loading points...");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  
  factory.registerNodeType<fpm_bt_tree_nodes::LoadPointsClient>("loadpoints");
  // factory.registerNodeType<fpm_bt_tree_nodes::LoadPointsClientDS2>("loadpointsDS2");

}