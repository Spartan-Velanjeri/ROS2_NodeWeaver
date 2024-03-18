// Copyright (c) 2024 Robert Bosch GmbH and its subsidiaries
//
// All rights reserved, also regarding any disposal, exploitation, reproduction,
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#include "fpm_bt_tree_nodes/actions/transform_clients.hpp"
namespace fpm_bt_tree_nodes
{
TransformDrillHole::TransformDrillHole(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
}

inline BT::NodeStatus TransformDrillHole::tick()
  {   
    
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("drill_pose_transform");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);  

    marker_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>("/drill_hole_marker", 1);
    getInput<geometry_msgs::msg::PoseStamped>("in_pose",in_pose);
    getInput<std::string>("in_target_frame_id",in_target_frame_id);
       
    geometry_msgs::msg::TransformStamped transformation;
    std::string fromFrameRel = in_pose.header.frame_id;
    std::string toFrameRel = in_target_frame_id;
    bool flag = true;
    while(flag)
    {
      try {
        transformation = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
        flag = false;
      } 
      catch (const tf2::TransformException & ex) {
        flag = true;
        RCLCPP_INFO(node->get_logger(), "Could not transform %s to %s: %s",toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
        setOutput("out_pose", in_pose);
        return BT::NodeStatus::FAILURE;
      }
    }

    // Simple pose transformation 
    tf2::doTransform(in_pose,out_pose,transformation);
    

    RCLCPP_INFO(node->get_logger(), "Input Drill Hole Position x: %f, y: %f, z: %f",in_pose.pose.position.x,in_pose.pose.position.y,in_pose.pose.position.z);
    RCLCPP_INFO(node->get_logger(), "Input Drill Hole Orientation x: %f, y: %f, z: %f, w: %f",in_pose.pose.orientation.x,in_pose.pose.orientation.y,in_pose.pose.orientation.z,in_pose.pose.orientation.w);
    RCLCPP_INFO(node->get_logger(), "Transformed Drill Hole Position x: %f, y: %f, z: %f",out_pose.pose.position.x,out_pose.pose.position.y,out_pose.pose.position.z);
    RCLCPP_INFO(node->get_logger(), "Transformed Drill Hole Orientation x: %f, y: %f, z: %f, w: %f",out_pose.pose.orientation.x,out_pose.pose.orientation.y,out_pose.pose.orientation.z,out_pose.pose.orientation.w);

    // the pose is sent to the blackboard here
    setOutput("out_pose", out_pose);

    // marker to visualize drill hole in rviz (not tested)
    visualization_msgs::msg::Marker marker_msg;
    marker_msg.action = visualization_msgs::msg::Marker::ADD;
    marker_msg.header.frame_id = "map";
    marker_msg.header.stamp = node->get_clock()->now();
    marker_msg.ns = "drill_goal_pose";
    marker_msg.type = visualization_msgs::msg::Marker::ARROW;;
    marker_msg.pose = in_pose.pose;
    marker_msg.pose.position.z = 0.1;
    marker_msg.color.r = 0.0;
    marker_msg.color.g = 0.5;
    marker_msg.color.b = 1.0;
    marker_msg.color.a = 0.8;
    marker_msg.scale.x = 0.4;
    marker_msg.scale.y = 0.1;
    marker_msg.scale.z = 0.1;
    marker_publisher_->publish(marker_msg);
    rclcpp::sleep_for(std::chrono::seconds(1));
    return BT::NodeStatus::SUCCESS;
  }

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  
  factory.registerNodeType<fpm_bt_tree_nodes::TransformDrillHole>("TransformDrillHole");
}