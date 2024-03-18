// Copyright (c) 2024 Robert Bosch GmbH and its subsidiaries
//
// All rights reserved, also regarding any disposal, exploitation, reproduction,
// editing, distribution, as well as in the event of applications for industrial
// property rights.
//
// @brief Nav2 navigator plugin for Behavior Tree (BT) skills.
//

#ifndef NAV2_BT_NAVIGATOR__BT_EXEC_SKILL_HPP_
#define NAV2_BT_NAVIGATOR__BT_EXEC_SKILL_HPP_

#include "bautiro_ros_interfaces/action/start_fpm_skill.hpp"
#include "fpm_behavior_tree/navigator.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <memory>
#include <string>
#include <vector>

namespace fpm_behavior_tree {

/**
 * @brief
 *
 */
class FpmSkillSequence : public fpm_behavior_tree::Navigator<
                             bautiro_ros_interfaces::action::StartFpmSkill> {
public:
  using ActionT = bautiro_ros_interfaces::action::StartFpmSkill;

  FpmSkillSequence() : Navigator() {}

  /**
   * @Method determines the blackboard parameter names where the goal and paths
   * are being stored, as these are key values for processing feedback in onLoop
   * and for the different behavior tree nodes to communicate this information
   * between themselves.
   *
   * @param node
   * @return true
   * @return false
   */
  bool configure(rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

  /**
   * @brief
   *
   * @return true
   * @return false
   */
  bool cleanup() override;

  std::string
  getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;
  std::string getDefaultBTFilename() override;
  std::string getName() { return std::string("fpm_skill_server"); }

protected:
  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;

  /**
   * @brief
   *
   */
  void onLoop() override;
  /**
   * @brief
   *
   * @param goal
   */
  void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;
  /**
   * @brief
   *
   * @param result
   */
  void goalCompleted(typename ActionT::Result::SharedPtr result) override;

  /**
   * @brief
   *
   */
  rclcpp::Time start_time_;

  /**
   * @brief
   *
   */
  rclcpp_action::Client<ActionT>::SharedPtr self_client_;

  /**
   * @brief
   *
   */
  std::string goal_blackboard_id_;
};

} // namespace fpm_behavior_tree

#endif