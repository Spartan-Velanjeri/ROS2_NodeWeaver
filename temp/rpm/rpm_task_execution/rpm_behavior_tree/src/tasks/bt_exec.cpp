//  Copyright 2023,2024 Robert Bosch GmbH and its subsidiaries
//
//  All rights reserved, also regarding any disposal, exploitation, reproduction,
//  editing, distribution, as well as in the event of applications for industrial
//  property rights.
#include <rpm_behavior_tree/tasks/bt_exec.hpp>
#include <std_msgs/msg/int32.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <vector>
#include <string>
#include <set>
#include <memory>
#include <limits>


namespace rpm_behavior_tree
{

bool
RpmMotionCluster::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)

{
  auto node = parent_node.lock();

  if (!node->has_parameter("goal_blackboard_id")) {
    node->declare_parameter("goal_blackboard_id", std::string("target_pose"));
  }
  goal_blackboard_id_ = node->get_parameter("goal_blackboard_id").as_string();


  self_client_ = rclcpp_action::create_client<ActionT>(node, getName());

  return true;
}

std::string
RpmMotionCluster::getDefaultBTFilepath(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  std::string package_path = ament_index_cpp::get_package_share_directory("rpm_behavior_tree");
  std::string default_bt_xml_filename;
  auto node = parent_node.lock();
  node->get_parameter("default_nav_to_pose_bt_xml", default_bt_xml_filename);
  default_bt_xml_filename = package_path + "/bt_xml/motion_nav2_p2p_upgrade.xml";
  return default_bt_xml_filename;
}

std::string
getDefaultXML()
{
  std::string default_bt_xml_filename_;
  default_bt_xml_filename_ = "/motion_nav2_p2p_upgrade.xml";
  return default_bt_xml_filename_;
}

bool
RpmMotionCluster::cleanup()
{
  self_client_.reset();
  return true;
}

bool
RpmMotionCluster::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
  std::string package_path = ament_index_cpp::get_package_share_directory("rpm_behavior_tree");
  auto bt_xml_filepath = package_path + "/bt_xml/";

  // Set behavior xml and load behavior tree
  if(goal->bt_xml.empty()){
    bt_xml_filepath += getDefaultXML();
    RCLCPP_WARN(logger_, "No XML request, defaulting to %s", bt_xml_filepath.c_str());
  } else{
    bt_xml_filepath += goal->bt_xml;
  }
  if (!bt_action_server_->loadBehaviorTree(bt_xml_filepath)) {
    return false;
  }

  RCLCPP_DEBUG(logger_, "the xml file %s is accepted", bt_xml_filepath.c_str());
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Pose  x: %f, y: %f, z: %f",
                                goal->des_cluster_pose.pks_pose.position.x,
                                goal->des_cluster_pose.pks_pose.position.y,
                                goal->des_cluster_pose.pks_pose.position.z);
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Orientation  x: %f, y: %f, z: %f, w: %f",
                                  goal->des_cluster_pose.pks_pose.orientation.x,
                                  goal->des_cluster_pose.pks_pose.orientation.y,
                                  goal->des_cluster_pose.pks_pose.orientation.z,
                                  goal->des_cluster_pose.pks_pose.orientation.w);
  // Assign action goal to blackboard
  auto blackboard = bt_action_server_->getBlackboard();
  // blackboard = bt_action_server_->getBlackboard();
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RPM_BT: Assign blackboard successful!");

  p.pose = goal->des_cluster_pose.pks_pose;
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RPM_BT: get Pose success!");

  blackboard->set<geometry_msgs::msg::PoseStamped>(goal_blackboard_id_, p);
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RPM_BT: set pose to blackboard sucess!");

  return true;
}

void
RpmMotionCluster::goalCompleted(ActionT::Result::SharedPtr result)
{
  result->message = "succeeded";
  RCLCPP_INFO(logger_, "%s", result->message.c_str());
}

void
RpmMotionCluster::onLoop()
{
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
  // RCLCPP_INFO(logger_, "the tree is running");
}
void
RpmMotionCluster::onPreempt(ActionT::Goal::ConstSharedPtr goal)
{
  RCLCPP_DEBUG(logger_, "%d", goal->start);
}

}  // namespace rpm_behavior_tree
