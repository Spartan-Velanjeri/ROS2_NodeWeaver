// Copyright (c) 2024 Robert Bosch GmbH and its subsidiaries
//
// All rights reserved, also regarding any disposal, exploitation, reproduction,
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#ifndef NAV2_BT_NAVIGATOR__BT_EXEC_HPP_
#define NAV2_BT_NAVIGATOR__BT_EXEC_HPP_

#include <string>
#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "fpm_behavior_tree/navigator.hpp"
//#include "nav2_bt_navigator/navigator.hpp"
#include "bautiro_ros_interfaces/action/start_fpm_main_task.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/odometry_utils.hpp"

namespace fpm_behavior_tree
{


class FpmDrillSequence
  : public fpm_behavior_tree::Navigator<bautiro_ros_interfaces::action::StartFpmMainTask>
{
public:
  using ActionT = bautiro_ros_interfaces::action::StartFpmMainTask;

  
  FpmDrillSequence()
  : Navigator() {}

  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

  
  bool cleanup() override;

  

  std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;
  std::string getDefaultBTFilename() override;
  std::string getName() {return std::string("fpm_start");}

  

protected:
  
  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;

 
  void onLoop() override;
  void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;
  
  void goalCompleted(typename ActionT::Result::SharedPtr result) override;

  

  rclcpp::Time start_time_;

  rclcpp_action::Client<ActionT>::SharedPtr self_client_;

  std::string goal_blackboard_id_;

};

}  

#endif  