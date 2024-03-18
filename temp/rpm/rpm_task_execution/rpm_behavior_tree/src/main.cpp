//  Copyright 2023, 2024 Robert Bosch GmbH and its subsidiaries
//
//  All rights reserved, also regarding any disposal, exploitation, reproduction,
//  editing, distribution, as well as in the event of applications for industrial
//  property rights.

#include <memory>

#include "rpm_behavior_tree/bt_navigation.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rpm_behavior_tree::BtNavigator>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
