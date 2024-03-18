#include <vector>
#include <string>
#include <set>
#include <memory>
#include <limits>
#include <hu_behavior_tree/tasks/bt_exec.hpp>
#include <std_msgs/msg/int32.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace hu_behavior_tree
{

bool
FpmDrillSequence::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
  
{
  auto node = parent_node.lock();
  if (!node->has_parameter("goal_blackboard_id")) {
    node->declare_parameter("goal_blackboard_id", std::string("goal"));
  }
  goal_blackboard_id_ = node->get_parameter("goal_blackboard_id").as_string();

  self_client_ = rclcpp_action::create_client<ActionT>(node, getName());

  return true;
}
std::string
FpmDrillSequence::getDefaultBTFilepath(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  std::string package_path = ament_index_cpp::get_package_share_directory("hu_behavior_tree");  
  std::string default_bt_xml_filename;
  auto node = parent_node.lock();
  node->get_parameter("default_nav_to_pose_bt_xml", default_bt_xml_filename);
  default_bt_xml_filename = package_path + "/bt_xml/dashboard_bt.xml";
  return default_bt_xml_filename;
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
  default_bt_xml_filename_ = "/dashboard_bt.xml";
  return default_bt_xml_filename_;
}


bool
FpmDrillSequence::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
  std::string package_path = ament_index_cpp::get_package_share_directory("hu_behavior_tree");  
  std::string bt_xml_filepath;

  // handle undefined xml
  if(goal->bt_xml.empty()){
    bt_xml_filepath = package_path + "/bt_xml/" + getDefaultXML();
    RCLCPP_INFO(logger_, "No XML file passed, defaulting to %s", bt_xml_filepath.c_str());
  }
  else{
    bt_xml_filepath = package_path + "/bt_xml/" + goal->bt_xml;
  }  if (!bt_action_server_->loadBehaviorTree(bt_xml_filepath)) {
    return false;
  }
  RCLCPP_INFO(logger_, "the xml file is accepted");
  auto blackboard = bt_action_server_->getBlackboard();
  // blackboard->set<std::vector<bautiro_ros_interfaces::msg::DrillMask>>(goal_blackboard_id_, goal->drill_masks);
  // blackboard->set<int32_t>("points_count_bb", goal->drill_masks.size());
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
  //auto feedback_msg = std::make_shared<ActionT::Feedback>();
  //auto blackboard = bt_action_server_->getBlackboard();
  //int current_val = 0;
  //int goales;
  //blackboard->get<int>("val_now", current_val);
  //blackboard->get<int>("goal_val", goales);
  //RCLCPP_INFO(logger_, "the goal from b is %d", goales);
  //RCLCPP_INFO(logger_, "the current value is %d", current_val);
  //feedback_utils_.y = current_val;
  //RCLCPP_INFO(logger_, "the updated x is %d", feedback_utils_.y);
  //feedback_msg->currentvalue = current_val;
  //RCLCPP_INFO(logger_, "the feedback is %d", feedback_msg->currentvalue);
  //bt_action_server_->publishFeedback(feedback_msg);
  // RCLCPP_INFO(logger_, "the tree is running");
}
void
FpmDrillSequence::onPreempt(ActionT::Goal::ConstSharedPtr goal)
{
  RCLCPP_INFO(logger_, "%d", goal->start);
}

}
