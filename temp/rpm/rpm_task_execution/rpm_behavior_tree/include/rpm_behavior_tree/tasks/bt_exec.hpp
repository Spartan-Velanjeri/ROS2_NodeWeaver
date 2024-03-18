//  Copyright 2023, 2024 Robert Bosch GmbH and its subsidiaries
//
//  All rights reserved, also regarding any disposal, exploitation, reproduction,
//  editing, distribution, as well as in the event of applications for industrial
//  property rights.

#ifndef RPM_BEHAVIOR_TREE__TASKS__BT_EXEC_HPP_
#define RPM_BEHAVIOR_TREE__TASKS__BT_EXEC_HPP_

#include <string>
#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rpm_behavior_tree/navigator.hpp"
#include "bautiro_ros_interfaces/action/start_rpm_main_task.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/odometry_utils.hpp"

namespace rpm_behavior_tree
{


class RpmMotionCluster
  : public rpm_behavior_tree::Navigator<bautiro_ros_interfaces::action::StartRpmMainTask>
{
public:
  using ActionT = bautiro_ros_interfaces::action::StartRpmMainTask;

  RpmMotionCluster()
  : Navigator() {}

  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;


  bool cleanup() override;



  std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;
  std::string getName() {return std::string("rpm_start");}



protected:
  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;


  void onLoop() override;
  void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;

  void goalCompleted(typename ActionT::Result::SharedPtr result) override;



  rclcpp::Time start_time_;

  rclcpp_action::Client<ActionT>::SharedPtr self_client_;

  std::string goal_blackboard_id_;
  std::string frame_blackboard_id_;
  // auto blackboard;
  geometry_msgs::msg::PoseStamped p;
  std::string f;
};

}  // namespace rpm_behavior_tree

#endif  // RPM_BEHAVIOR_TREE__TASKS__BT_EXEC_HPP_
