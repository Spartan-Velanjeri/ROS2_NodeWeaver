#include "ccu_behavior_tree/bt_ccu_executor.hpp"

namespace ccu_behavior_tree
{

CcuBtExecution::CcuBtExecution()
: nav2_util::LifecycleNode("ccu_bt", "", false)
{
  RCLCPP_INFO(get_logger(), "Creating");

  const std::vector<std::string> plugin_libs = {
    "data_service_client_nodes",
    "rpm_client_nodes",
    "fpm_client_nodes",
  };
  declare_parameter("plugin_lib_names", plugin_libs);
  declare_parameter("x", 10);
  declare_parameter("y", 0);
}

CcuBtExecution::~CcuBtExecution()
{
}

nav2_util::CallbackReturn
CcuBtExecution::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  x_ = get_parameter("x").as_int();
  y_ = get_parameter("y").as_int();
  // Libraries to pull plugins (BT Nodes) from
  auto plugin_lib_names = get_parameter("plugin_lib_names").as_string_array();

  first_navigator_ = std::make_unique<ccu_behavior_tree::CcuCluster>();
  //second_navigator_ = std::make_unique<bt_server::Subtract>();
  ccu_behavior_tree::FeedbackUtils feedback_utils;
  feedback_utils.x = x_;
  feedback_utils.y = y_;

  if (!first_navigator_->on_configure(
      shared_from_this(), plugin_lib_names,feedback_utils, &plugin_muxer_))
  {
    return nav2_util::CallbackReturn::FAILURE;
  }
  //if (!second_navigator_->on_configure(
    //  shared_from_this(), plugin_lib_names,feedback_utils, &plugin_muxer_))
 // {
   // return nav2_util::CallbackReturn::FAILURE;
  //}


  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CcuBtExecution::on_activate(const rclcpp_lifecycle::State & /*state*/)
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
CcuBtExecution::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
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
CcuBtExecution::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // Reset the listener before the buffer

  if (!first_navigator_->on_cleanup()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  first_navigator_.reset();
  //second_navigator_.reset();

  RCLCPP_INFO(get_logger(), "Completed Cleaning up");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CcuBtExecution::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

} 