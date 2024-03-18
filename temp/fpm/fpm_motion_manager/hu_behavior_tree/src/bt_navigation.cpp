#include "hu_behavior_tree/bt_navigation.hpp"

#include <memory>
#include <string>
#include <utility>
#include <set>
#include <limits>
#include <vector>

#include <nav2_util/geometry_utils.hpp>
#include <nav2_util/robot_utils.hpp>
#include <nav2_behavior_tree/bt_conversions.hpp>

namespace hu_behavior_tree
{

BtNavigator::BtNavigator()
: nav2_util::LifecycleNode("hu_bt", "", false)
{
  RCLCPP_INFO(get_logger(), "Creating");

  const std::vector<std::string> plugin_libs = {
    "hu_motion_manager_clients_node",
    "hu_point_handler_node",
    "hu_dashboard_clients_node",
    "hu_io_and_status_clients_node",
    "hu_moveit_clients_node"
    };
  declare_parameter("plugin_lib_names", plugin_libs);
  declare_parameter("x", 10);
  declare_parameter("y", 0);
  declare_parameter("groot_zmq_publisher_port", 2200);
  declare_parameter("groot_zmq_server_port", 2201);
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

  hu_application = std::make_unique<hu_behavior_tree::FpmDrillSequence>();
  hu_behavior_tree::FeedbackUtils feedback_utils;
  feedback_utils.x = x_;
  feedback_utils.y = y_;
  if (!hu_application->on_configure(
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

  if (!hu_application->on_activate()) {
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

  if (!hu_application->on_deactivate()) {
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

  if (!hu_application->on_cleanup()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  hu_application.reset();

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
