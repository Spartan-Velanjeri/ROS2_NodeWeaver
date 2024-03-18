// Copyright (c) 2024 Robert Bosch GmbH and its subsidiaries
//
// All rights reserved, also regarding any disposal, exploitation, reproduction,
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#include <string>
#include <memory>
#include "behaviortree_cpp_v3/action_node.h"
#include "fpm_bt_tree_nodes/actions/get_poi.hpp"

#include <chrono>
 

using namespace std::chrono_literals; 
geometry_msgs::msg::PoseStamped p;
// int drillholes[100];

namespace fpm_bt_tree_nodes
{
// struct pattern {
//   struct drillholes[];
//   };
// struct drillholes {
//   geometry_msgs:msg::Pose p;

// };
GetPoi::GetPoi(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  
}

inline BT::NodeStatus GetPoi::tick()
{
  
      std::shared_ptr<rclcpp::Node> node_ = rclcpp::Node::make_shared("load_points");
      std::vector<bautiro_ros_interfaces::msg::DrillMask> array;
      geometry_msgs::msg::Pose p, q;
      int32_t i = 0;
    //   auto blackboard = BT::Blackboard::create();
    //   total = blackboard->get("points_count_bb", total);
      getInput<int>("total",total);
      getInput<int32_t>("point_index", i);
      getInput<std::vector<bautiro_ros_interfaces::msg::DrillMask>>("point_list", array);
      getInput<int>("points_count_bb",total);
    //   getInput<std::vector<bautiro_ros_interfaces::msg::Pattern>>("currentpoint", posesbb);
    //   getInput<geometry_msgs::msg::PoseArray>("currentpoint", posesbb);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "the current index is: %d ", i);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "the total count is: %d ", total);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pose Found x: %f y: %f z: %f rx: %f ry: %f rz: %f rw: %f", array[i].drill_holes[0].pks_pose.position.x,
                                                                                                            array[i].drill_holes[0].pks_pose.position.y, 
                                                                                                            array[i].drill_holes[0].pks_pose.position.z, 
                                                                                                            array[i].drill_holes[0].pks_pose.orientation.x, 
                                                                                                            array[i].drill_holes[0].pks_pose.orientation.y, 
                                                                                                            array[i].drill_holes[0].pks_pose.orientation.z, 
                                                                                                            array[i].drill_holes[0].pks_pose.orientation.w);
      p.position.x = array[i].drill_holes[0].pks_pose.position.x;
      p.position.y = array[i].drill_holes[0].pks_pose.position.y;
      p.position.z = array[i].drill_holes[0].pks_pose.position.z;
      p.orientation.x = array[i].drill_holes[0].pks_pose.orientation.x;
      p.orientation.y = array[i].drill_holes[0].pks_pose.orientation.y;
      p.orientation.z = array[i].drill_holes[0].pks_pose.orientation.z;
      p.orientation.w = array[i].drill_holes[0].pks_pose.orientation.w;
      
      q.position.x = array[i].drill_holes[0].pks_pose.position.x;
      q.position.y = array[i].drill_holes[0].pks_pose.position.y;
      q.position.z = array[i].drill_holes[0].pks_pose.position.z + 0.07;
      q.orientation.x = array[i].drill_holes[0].pks_pose.orientation.x;
      q.orientation.y = array[i].drill_holes[0].pks_pose.orientation.y;
      q.orientation.z = array[i].drill_holes[0].pks_pose.orientation.z;
      q.orientation.w = array[i].drill_holes[0].pks_pose.orientation.w;
      setOutput("drill_point", p);
      setOutput("end_point", q);
      return BT::NodeStatus::SUCCESS;
}



}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  
  factory.registerNodeType<fpm_bt_tree_nodes::GetPoi>("getpoi");
}