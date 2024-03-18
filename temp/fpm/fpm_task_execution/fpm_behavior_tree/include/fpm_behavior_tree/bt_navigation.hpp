// Copyright (c) 2024 Robert Bosch GmbH and its subsidiaries
//
// All rights reserved, also regarding any disposal, exploitation, reproduction,
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#ifndef NAV2_BT_NAVIGATOR__BT_NAVIGATOR_HPP_
#define NAV2_BT_NAVIGATOR__BT_NAVIGATOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "fpm_behavior_tree/tasks/bt_exec.hpp"
//#include "bt_server/bt_sub.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fpm_behavior_tree
{


class BtNavigator : public nav2_util::LifecycleNode
{
public:
  
  BtNavigator();
 
  ~BtNavigator();
protected:
  
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
 

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  // To handle all the BT related execution
  // std::unique_ptr<fpm_behavior_tree::Navigator<bautiro_ros_interfaces::action::StartFpmMainTask>> fpm_drill_application_sim;
  std::unique_ptr<fpm_behavior_tree::Navigator<bautiro_ros_interfaces::action::StartFpmMainTask>> fpm_drill_application_dep; 
  fpm_behavior_tree::NavigatorMuxer plugin_muxer_;
  int x_;
  int y_;

};

}  

#endif  // NAV2_BT_NAVIGATOR__BT_NAVIGATOR_HPP_