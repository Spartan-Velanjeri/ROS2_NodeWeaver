//  Copyright 2023, 2024 Robert Bosch GmbH and its subsidiaries
//
//  All rights reserved, also regarding any disposal, exploitation, reproduction,
//  editing, distribution, as well as in the event of applications for industrial
//  property rights.

#include "rpm_behavior_tree/bt_navigation.hpp"
#include <nav2_util/geometry_utils.hpp>
#include <nav2_util/robot_utils.hpp>
#include <nav2_behavior_tree/bt_conversions.hpp>
#include <memory>
#include <string>
#include <utility>
#include <set>
#include <limits>
#include <vector>



namespace rpm_behavior_tree
{

BtNavigator::BtNavigator()
: nav2_util::LifecycleNode("rpm_bt", "", false)
{
  RCLCPP_INFO(get_logger(), "Creating");

  const std::vector<std::string> plugin_libs = {
    "move_p2p_async_node",
    "nav2_pose_node",
    "switch_point_async_node",
    "transform_pose_async_node"
  };
  declare_parameter("plugin_lib_names", plugin_libs);
  declare_parameter("x", 10);
  declare_parameter("y", 0);
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

  first_navigator_ = std::make_unique<rpm_behavior_tree::RpmMotionCluster>();
  // second_navigator_ = std::make_unique<bt_server::Subtract>();
  rpm_behavior_tree::FeedbackUtils feedback_utils;
  feedback_utils.x = x_;
  feedback_utils.y = y_;

  if (!first_navigator_->on_configure(
      shared_from_this(), plugin_lib_names, feedback_utils, &plugin_muxer_))
  {
    return nav2_util::CallbackReturn::FAILURE;
  }
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigator::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  if (!first_navigator_->on_activate()) {
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

  if (!first_navigator_->on_deactivate()) {
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

  if (!first_navigator_->on_cleanup()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  first_navigator_.reset();


  RCLCPP_INFO(get_logger(), "Completed Cleaning up");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigator::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

}  // namespace rpm_behavior_tree
