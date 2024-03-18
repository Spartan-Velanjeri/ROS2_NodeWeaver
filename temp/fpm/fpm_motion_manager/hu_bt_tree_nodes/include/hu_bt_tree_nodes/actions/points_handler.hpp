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
namespace hu_bt_tree_nodes
{


class GetPoi : public BT::ActionNodeBase
{
public:
  rclcpp::Node::SharedPtr node_;


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
  geometry_msgs::msg::PoseStamped p;
// int drillholes[100];


};

}  // namespace hu_bt_tree_nodes

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__BACK_UP_ACTION_HPP_SIM_


#ifndef BEHAVIOR_TREE__PLUGINS__ACTION__FIFTH_NODE_HPP
#define BEHAVIOR_TREE__PLUGINS__ACTION__FIFTH_NODE_HPP

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <behaviortree_cpp_v3/action_node.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <yaml-cpp/yaml.h>
#include <algorithm>  // std::min
#include <vector>
#include <atomic>
#include <memory>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "std_msgs/msg/int32.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>


namespace hu_bt_tree_nodes
{


class UpdatePoi : public BT::ActionNodeBase
{
public:
  rclcpp::Node::SharedPtr node_;

  UpdatePoi(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;
  void halt() override {}
  static BT::PortsList providedPorts()
  {
    return
      {
        BT::InputPort<int32_t>("total"), 
        BT::InputPort<int32_t>("point_index"), 
        BT::OutputPort<int32_t>("update_index")
      };
  }
  int initial, total;
};

}  // namespace hu_bt_tree_nodes

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__BACK_UP_ACTION_HPP_SIM_
