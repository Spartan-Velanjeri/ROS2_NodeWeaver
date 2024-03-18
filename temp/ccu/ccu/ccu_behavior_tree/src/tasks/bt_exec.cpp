/* Copyright 2023 Robert Bosch GmbH and its subsidiaries

 All rights reserved, also regarding any disposal, exploitation, reproduction
 editing, distribution, as well as in the event of applications for industrial
 property rights. */





//BAUTIRO
#include <ccu_behavior_tree/tasks/bt_exec.hpp>

namespace ccu_behavior_tree
{

bool
CcuCluster::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
  
{
  auto node = parent_node.lock();

  if (!node->has_parameter("goal_blackboard_id")) {
    node->declare_parameter("goal_blackboard_id", std::string("goal_val"));
  }
  goal_blackboard_id_ = node->get_parameter("goal_blackboard_id").as_string();

  self_client_ = rclcpp_action::create_client<ActionT>(node, getName());

  return true;
}
std::string
CcuCluster::getDefaultBTFilepath(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  std::string package_path = ament_index_cpp::get_package_share_directory("ccu_behavior_tree");  
  std::string default_bt_xml_filepath;
  auto node = parent_node.lock();
  node->get_parameter("default_nav_to_pose_bt_xml", default_bt_xml_filepath);
  default_bt_xml_filepath = package_path + "/bt_xml/execute_mission.xml";
  return default_bt_xml_filepath;
}

// sets the default file name to use if no xml file is selected
std::string 
getDefaultXML()
  {
    std::string default_bt_xml_file = "ccu_main_feedback.xml";
      RCLCPP_INFO(rclcpp::get_logger("Navigator"), "No XML file passed, defaulting to %s", default_bt_xml_file.c_str());
    return default_bt_xml_file;
  }

bool
CcuCluster::cleanup()
{
  self_client_.reset();
  return true;
}

bool
CcuCluster::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
  std::string package_path = ament_index_cpp::get_package_share_directory("ccu_behavior_tree");  
  std::string bt_xml_filepath;

  // handle undefined xml
  if(goal->bt_xml.empty()){
    bt_xml_filepath = package_path + "/bt_xml/" + getDefaultXML();
    RCLCPP_INFO(logger_, "No XML file passed, defaulting to %s", bt_xml_filepath.c_str());
  }
  else{
    bt_xml_filepath = package_path + "/bt_xml/" + goal->bt_xml;
  }


  if (!bt_action_server_->loadBehaviorTree(bt_xml_filepath)) {
    return false;
  }
  RCLCPP_INFO(logger_, "the xml file is accepted");
  auto blackboard = bt_action_server_->getBlackboard();
  //int goals;
  // Update the goal pose on the blackboard
  blackboard->set<bool>(goal_blackboard_id_, goal->start);

  return true;
}

void
CcuCluster::goalCompleted(ActionT::Result::SharedPtr result)
{
  result->message = "succeeded";
  RCLCPP_INFO(logger_, "%s", result->message.c_str());
}

void
CcuCluster::onLoop()
{
}
void
CcuCluster::onPreempt(ActionT::Goal::ConstSharedPtr goal)
{
  // handle new goals during execution
  if(goal->start){
    RCLCPP_WARN(
        logger_,
        "Rejecting new CCU Goal as current goal is still executing.");
      bt_action_server_->terminatePendingGoal();
    }
  }

}
