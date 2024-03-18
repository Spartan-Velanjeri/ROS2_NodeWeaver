// Copyright (c) 2024 Robert Bosch GmbH and its subsidiaries
//
// All rights reserved, also regarding any disposal, exploitation, reproduction,
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#include "fpm_behavior_tree/bt_fpm_skill.hpp"

#include <limits>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <nav2_behavior_tree/bt_conversions.hpp>
#include <nav2_util/geometry_utils.hpp>
#include <nav2_util/robot_utils.hpp>

namespace fpm_behavior_tree {

BtFpmSkill::BtFpmSkill() : nav2_util::LifecycleNode("fpm_bt_skill", "", false) {
  RCLCPP_INFO(get_logger(), "Creating fpm skill server...");

  const std::vector<std::string> plugin_libs = {
      "lift_node",
      "hu_bt_node",
      "hu_mm_node",
      "lu_fine_node",
      "move_lift_absolute_client_node",
      //"move_absolute_client_sim_node",
      //"move_lift_absolute_client_sim_node",
      "update_poi_node",
      "get_poi_node",
      "load_point_node",
  };
  declare_parameter("plugin_lib_names", plugin_libs);
  declare_parameter("x", 10);
  declare_parameter("y", 0);
  declare_parameter("groot_zmq_publisher_port", 2100);
  declare_parameter("groot_zmq_server_port", 2101);
}

BtFpmSkill::~BtFpmSkill() {}

nav2_util::CallbackReturn
BtFpmSkill::on_configure(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(get_logger(), "BtFpmSkil: Configuring");
  x_ = get_parameter("x").as_int();
  y_ = get_parameter("y").as_int();
  // Libraries to pull plugins (BT Nodes) from
  auto plugin_lib_names = get_parameter("plugin_lib_names").as_string_array();

  fpm_skill = std::make_unique<fpm_behavior_tree::FpmSkillSequence>();
  fpm_behavior_tree::FeedbackUtils feedback_utils;
  feedback_utils.x = x_;
  feedback_utils.y = y_;
  if (!fpm_skill->on_configure(shared_from_this(), plugin_lib_names,
                               feedback_utils, &plugin_muxer_)) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtFpmSkill::on_activate(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(get_logger(), "BtFpmSkil: Activating");

  if (!fpm_skill->on_activate()) {
    return nav2_util::CallbackReturn::FAILURE;
  }
  // create bond connection
  createBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtFpmSkill::on_deactivate(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(get_logger(), "BtFpmSkil: Deactivating");

  if (!fpm_skill->on_deactivate()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtFpmSkill::on_cleanup(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(get_logger(), "BtFpmSkil: Cleaning up.");

  // Reset the listener before the buffer

  if (!fpm_skill->on_cleanup()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  fpm_skill.reset();

  RCLCPP_INFO(get_logger(), "BtFpmSkil: Completed Cleaning up.");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtFpmSkill::on_shutdown(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(get_logger(), "BtFpmSkil: Shutting down BtFpmSkillNavigator...");
  return nav2_util::CallbackReturn::SUCCESS;
}
////////////////////////////////////

} // namespace fpm_behavior_tree

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fpm_behavior_tree::BtFpmSkill>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}