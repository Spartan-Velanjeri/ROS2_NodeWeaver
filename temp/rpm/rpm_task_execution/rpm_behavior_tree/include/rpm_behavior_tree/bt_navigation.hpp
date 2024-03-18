//  Copyright 2023, 2024 Robert Bosch GmbH and its subsidiaries
//
//  All rights reserved, also regarding any disposal, exploitation, reproduction,
//  editing, distribution, as well as in the event of applications for industrial
//  property rights.

#ifndef RPM_BEHAVIOR_TREE__BT_NAVIGATION_HPP_
#define RPM_BEHAVIOR_TREE__BT_NAVIGATION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rpm_behavior_tree/tasks/bt_exec.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rpm_behavior_tree
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
  std::unique_ptr<rpm_behavior_tree::Navigator<
        bautiro_ros_interfaces::action::StartRpmMainTask>> first_navigator_;

  rpm_behavior_tree::NavigatorMuxer plugin_muxer_;
  int x_;
  int y_;
};

}   // namespace rpm_behavior_tree

#endif  // RPM_BEHAVIOR_TREE__BT_NAVIGATION_HPP_
