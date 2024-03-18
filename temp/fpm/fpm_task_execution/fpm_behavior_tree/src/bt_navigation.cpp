// Copyright (c) 2024 Robert Bosch GmbH and its subsidiaries
//
// All rights reserved, also regarding any disposal, exploitation, reproduction,
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#include "fpm_behavior_tree/bt_navigation.hpp"

#include <memory>
#include <string>
#include <utility>
#include <set>
#include <limits>
#include <vector>

#include <nav2_util/geometry_utils.hpp>
#include <nav2_util/robot_utils.hpp>
#include <nav2_behavior_tree/bt_conversions.hpp>

namespace fpm_behavior_tree
{

BtNavigator::BtNavigator()
: nav2_util::LifecycleNode("fpm_bt_navigators", "", false)
{
  RCLCPP_INFO(get_logger(), "Creating");

  const std::vector<std::string> plugin_libs = {
    "drill_hole_transform_node",
    "configured_pose_client_node",
    "move_absolute_client_node",
    "lift_node",
    "hu_node",
    "move_lift_absolute_client_node",
    "lu_get_marker_rough_position_node",
    "ts_measure_point_node",
    "lu_triangulation_node",
    "lu_map_leica_tf_node",
    "lu_get_target_pose_ur_frame_node",
    "lu_base_link_desired_tcp_tf_node",
    "move_absolute_client_sim_node",
    "move_lift_absolute_client_sim_node",
    "update_poi_node",
    "get_poi_node",
    "load_point_node"
    };
  declare_parameter("plugin_lib_names", plugin_libs);
  declare_parameter("x", 10);
  declare_parameter("y", 0);
  declare_parameter("groot_zmq_publisher_port", 2100);
  declare_parameter("groot_zmq_server_port", 2101);
}

BtNavigator::~BtNavigator()
{
}

nav2_util::CallbackReturn
BtNavigator::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  x_ = get_parameter("x").as_int();
  y_ = get_parameter("y").as_int();
  // Libraries to pull plugins (BT Nodes) from
  auto plugin_lib_names = get_parameter("plugin_lib_names").as_string_array();

  fpm_drill_application_dep = std::make_unique<fpm_behavior_tree::FpmDrillSequence>();
  fpm_behavior_tree::FeedbackUtils feedback_utils;
  feedback_utils.x = x_;
  feedback_utils.y = y_;
  if (!fpm_drill_application_dep->on_configure(
      shared_from_this(), plugin_lib_names,feedback_utils, &plugin_muxer_))
  {
    return nav2_util::CallbackReturn::FAILURE;
  }
  

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigator::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  if (!fpm_drill_application_dep->on_activate()) {
    return nav2_util::CallbackReturn::FAILURE;
  }
  // create bond connection
  createBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigator::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  if (!fpm_drill_application_dep->on_deactivate()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigator::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // Reset the listener before the buffer

  if (!fpm_drill_application_dep->on_cleanup()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  fpm_drill_application_dep.reset();

  RCLCPP_INFO(get_logger(), "Completed Cleaning up");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigator::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}
////////////////////////////////////
} 
