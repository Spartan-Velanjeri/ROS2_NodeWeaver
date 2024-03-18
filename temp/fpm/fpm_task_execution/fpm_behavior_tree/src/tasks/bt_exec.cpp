// Copyright (c) 2024 Robert Bosch GmbH and its subsidiaries
//
// All rights reserved, also regarding any disposal, exploitation, reproduction,
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#include <vector>
#include <string>
#include <set>
#include <memory>
#include <limits>
#include <fpm_behavior_tree/tasks/bt_exec.hpp>
#include <std_msgs/msg/int32.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace fpm_behavior_tree
{

bool
FpmDrillSequence::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
  
{
  auto node = parent_node.lock();
  if (!node->has_parameter("goal_blackboard_id")) {
    node->declare_parameter("goal_blackboard_id", std::string("drill_masks"));
  }
  goal_blackboard_id_ = node->get_parameter("goal_blackboard_id").as_string();

  self_client_ = rclcpp_action::create_client<ActionT>(node, getName());

  return true;
}
std::string
FpmDrillSequence::getDefaultBTFilepath(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  std::string package_path = ament_index_cpp::get_package_share_directory("fpm_behavior_tree");  
  std::string default_bt_xml_filename;
  auto node = parent_node.lock();
  node->get_parameter("default_nav_to_pose_bt_xml", default_bt_xml_filename);
  default_bt_xml_filename = package_path + "/bt_xml/lu_fpm.xml";
  return default_bt_xml_filename;
}
std::string 
FpmDrillSequence::getDefaultBTFilename()
  {
    std::string default_bt_xml_filename_;
    default_bt_xml_filename_ = "lu_fpm.xml";
    return default_bt_xml_filename_;
  }

bool
FpmDrillSequence::cleanup()
{
  self_client_.reset();
  return true;
}
std::string 
getDefaultXML()
{
  std::string default_bt_xml_filename_;
  default_bt_xml_filename_ = "lu_fpm.xml";
  return default_bt_xml_filename_;
}


bool
FpmDrillSequence::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
  std::string package_path = ament_index_cpp::get_package_share_directory("fpm_behavior_tree");  
  auto bt_xml_filepath = package_path + "/bt_xml/";
  if(goal->bt_xml.empty()){
    bt_xml_filepath += bt_action_server_->getDefaultBTFilename();
  }
  else{
    bt_xml_filepath += goal->bt_xml;
  }
  RCLCPP_INFO(logger_, package_path.c_str());
  RCLCPP_INFO(logger_, "------------");
  RCLCPP_INFO(logger_, bt_xml_filepath.c_str());
  if (!bt_action_server_->loadBehaviorTree(bt_xml_filepath)) {
    return false;
  }
  RCLCPP_INFO(logger_, "the xml file is accepted");
  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<std::vector<bautiro_ros_interfaces::msg::DrillMask>>(goal_blackboard_id_, goal->drill_masks);
  blackboard->set<int32_t>("points_count_bb", goal->drill_masks.size());
  return true;
}

void
FpmDrillSequence::goalCompleted(ActionT::Result::SharedPtr result)
{
  result->message = "succeeded";
  RCLCPP_INFO(logger_, "%s", result->message.c_str());
}

void
FpmDrillSequence::onLoop()
{
//   auto feedback_msg = std::make_shared<ActionT::Feedback>();
//   auto blackboard = bt_action_server_->getBlackboard();
//   int current_val = 0;
//   int goales;
//   blackboard->get<int>("val_now", current_val);
//   blackboard->get<int>("goal_val", goales);
//   RCLCPP_INFO(logger_, "the goal from b is %d", goales);
//   RCLCPP_INFO(logger_, "the current value is %d", current_val);
//   feedback_utils_.y = current_val;
//   RCLCPP_INFO(logger_, "the updated x is %d", feedback_utils_.y);
//   feedback_msg->currentvalue = current_val;
//   RCLCPP_INFO(logger_, "the feedback is %d", feedback_msg->currentvalue);
//   bt_action_server_->publishFeedback(feedback_msg);
//   RCLCPP_INFO(logger_, "the tree is running");
}
void
FpmDrillSequence::onPreempt(ActionT::Goal::ConstSharedPtr goal)
{
  RCLCPP_INFO(logger_, "%d", goal->start);
}

}
