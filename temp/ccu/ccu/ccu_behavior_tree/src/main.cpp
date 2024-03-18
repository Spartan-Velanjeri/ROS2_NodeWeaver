/* Copyright 2023 Robert Bosch GmbH and its subsidiaries

 All rights reserved, also regarding any disposal, exploitation, reproduction
 editing, distribution, as well as in the event of applications for industrial
 property rights. */

//common
#include <memory>


//ROS
#include "ccu_behavior_tree/bt_ccu_executor.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ccu_behavior_tree::CcuBtExecution>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}