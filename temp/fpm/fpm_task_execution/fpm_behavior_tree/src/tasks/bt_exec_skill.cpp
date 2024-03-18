// Copyright (c) 2024 Robert Bosch GmbH and its subsidiaries
//
// All rights reserved, also regarding any disposal, exploitation, reproduction,
// editing, distribution, as well as in the event of applications for industrial
// property rights.

// noqa

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <vector>
#include "fpm_behavior_tree/tasks/bt_exec_skill.hpp"
#include <std_msgs/msg/float32.hpp>
#include "std_msgs/msg/int32.hpp"

namespace fpm_behavior_tree {

bool FpmSkillSequence::configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)

{
  RCLCPP_INFO(logger_, "FpmSkillSequence: configure.");
  auto node = parent_node.lock();
  // if (!node->has_parameter("goal_blackboard_id")) {
  //   node->declare_parameter("goal_blackboard_id",
  //   std::string("drill_masks"));
  // }
  // goal_blackboard_id_ =
  // node->get_parameter("goal_blackboard_id").as_string();

  self_client_ = rclcpp_action::create_client<ActionT>(node, getName());

  return true;
}

std::string FpmSkillSequence::getDefaultBTFilepath(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node) {

  // FIXME: Why default is needed and which should be loaded.
  RCLCPP_INFO(logger_, "FpmSkillSequence: getDefaultBTFilepath.");
  std::string package_path =
      ament_index_cpp::get_package_share_directory("fpm_behavior_tree");
  std::string default_bt_xml_filename;
  auto node = parent_node.lock();
  node->get_parameter("default_nav_to_pose_bt_xml", default_bt_xml_filename);
  default_bt_xml_filename = package_path + "/bt_xml/fpm_drill_pattern.xml";
  return default_bt_xml_filename;
}

std::string
FpmSkillSequence::getDefaultBTFilename()
  {
    std::string default_bt_xml_filename_;
    default_bt_xml_filename_ = "/lu_fpm.xml";
    return default_bt_xml_filename_;
  }

bool FpmSkillSequence::cleanup() {
  RCLCPP_INFO(logger_, "FpmSkillSequence: cleanup.");
  self_client_.reset();
  return true;
}
// std::string
// getDefaultXML()
// {
//   std::string default_bt_xml_filename_;
//   default_bt_xml_filename_ = "/demo_cluster.xml";
//   return default_bt_xml_filename_;
// }

bool FpmSkillSequence::goalReceived(ActionT::Goal::ConstSharedPtr goal) {
  RCLCPP_INFO(logger_, "FpmSkillSequence: goalReceived.");
  std::string package_path =
      ament_index_cpp::get_package_share_directory("fpm_behavior_tree");
  auto bt_xml_filename = package_path + "/bt_xml/" + goal->skill_name;

  if (!bt_action_server_->loadBehaviorTree(bt_xml_filename)) {
    return false;
  }
  RCLCPP_INFO(logger_, "Skill <%s> xml file is loaded.",
              goal->skill_name.c_str());

  // write goal to blackboard
  auto blackboard = bt_action_server_->getBlackboard();
  if (goal->skill_name == "fpm_lift_move_absolute.xml") {
    RCLCPP_INFO(logger_, "Writing lift_setpoint_position:%f to blackboard.",
                goal->lift_setpoint_position);

    blackboard->set<float>("lift_setpoint_position",
                           goal->lift_setpoint_position);

    return true;
  } else if (goal->skill_name == "fpm_handlig_unit_move_to_pose.xml") {
    RCLCPP_INFO(logger_,
                "Writing handling_unit_configured_pose:%d to blackboard.",
                goal->handling_unit_configured_pose);

    blackboard->set<int>("handling_unit_configured_pose",
                         goal->handling_unit_configured_pose);

    return true;
  } else if (goal->skill_name == "fpm_lift_move_to_working_height.xml") {
    RCLCPP_INFO(logger_, "Writing platform_ceiling_distance:%f to blackboard.",
                goal->platform_ceiling_distance);

    blackboard->set<float>("platform_ceiling_distance", goal->platform_ceiling_distance);

    return true;
  } else if (goal->skill_name == "fpm_drill_manually.xml") {
    RCLCPP_INFO(logger_, "Writing platform_ceiling_distance:%f to blackboard.",
                goal->platform_ceiling_distance);

    blackboard->set<float>("platform_ceiling_distance", goal->platform_ceiling_distance);

    return true;
  }

  RCLCPP_WARN(logger_, "Unknown skill <%s> called.", goal->skill_name.c_str());
  blackboard->set<float>("lift_setpoint_position",
                           goal->lift_setpoint_position);
  blackboard->set<int>("handling_unit_configured_pose",
                         goal->handling_unit_configured_pose);
  blackboard->set<float>("platform_ceiling_distance", goal->platform_ceiling_distance);
  return true;
}
// namespace fpm_behavior_tree

void FpmSkillSequence::goalCompleted(ActionT::Result::SharedPtr result) {
  RCLCPP_INFO(logger_, "FpmSkillSequence: goalCompleted.");
  // result->out_payload = "Test payload";
  RCLCPP_INFO(logger_, "FpmSkillSequence response_code %d",
              result->response_code);
}

void FpmSkillSequence::onLoop() {
  RCLCPP_INFO(logger_, "FpmSkillSequence: onLoop.");
  // auto feedback_msg = std::make_shared<ActionT::Feedback>();
  // auto blackboard = bt_action_server_->getBlackboard();
  // int current_val = 0;
  // int goales;
  // blackboard->get<int>("val_now", current_val);
  // blackboard->get<int>("goal_val", goales);
  // RCLCPP_INFO(logger_, "the goal from b is %d", goales);
  // RCLCPP_INFO(logger_, "the current value is %d", current_val);
  // feedback_utils_.y = current_val;
  // RCLCPP_INFO(logger_, "the updated x is %d", feedback_utils_.y);
  // feedback_msg->currentvalue = current_val;
  // RCLCPP_INFO(logger_, "the feedback is %d", feedback_msg->currentvalue);
  // bt_action_server_->publishFeedback(feedback_msg);
  //  RCLCPP_INFO(logger_, "the tree is running");
}

void FpmSkillSequence::onPreempt(ActionT::Goal::ConstSharedPtr goal) {
  RCLCPP_INFO(logger_, "onPreempt called.");
  RCLCPP_INFO(logger_, "%s", goal->skill_name.c_str());
  // RCLCPP_INFO(logger_, "%d", goal->start);
}

} // namespace fpm_behavior_tree
