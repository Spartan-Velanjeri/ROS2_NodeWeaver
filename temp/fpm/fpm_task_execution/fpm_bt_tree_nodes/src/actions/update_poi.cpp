// Copyright (c) 2024 Robert Bosch GmbH and its subsidiaries
//
// All rights reserved, also regarding any disposal, exploitation, reproduction,
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#include <string>
#include <memory>
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
#include "behaviortree_cpp_v3/action_node.h"
#include "fpm_bt_tree_nodes/actions/update_poi.hpp"

namespace fpm_bt_tree_nodes
{

UpdatePoi::UpdatePoi(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{

}

inline BT::NodeStatus UpdatePoi::tick()
{
  
      int total;
      int32_t i;
    //   auto blackboard = BT::Blackboard::create();
    //   total = blackboard->get("points_count_bb", total);
      getInput<int>("total",total);
      getInput<int>("point_index", i);
      i = i+1;
      setOutput("update_index", i);
    //   blackboard->set("point_index", i);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "the current index is: %d ", i);
      
      return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  
  factory.registerNodeType<fpm_bt_tree_nodes::UpdatePoi>("updatepoi");
}