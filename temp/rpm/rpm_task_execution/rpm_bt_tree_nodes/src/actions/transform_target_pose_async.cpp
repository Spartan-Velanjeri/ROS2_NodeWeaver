//  Copyright 2024 Robert Bosch GmbH and its subsidiaries
//
//  All rights reserved, also regarding any disposal, exploitation, reproduction,
//  editing, distribution, as well as in the event of applications for industrial
//  property rights.

#include "rpm_bt_tree_nodes/actions/transform_target_pose_async.hpp"

namespace rpm_behavior_tree
{ bool check;
  geometry_msgs::msg::PoseStamped in_pose, out_pose;

  BT::NodeStatus Transform_target_async::onStart()
  {
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("goal_pose_transform");
    RCLCPP_DEBUG(node->get_logger(), "i am in transform goal!!!");
    check = false ;

     if (!node)
    {
       RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "node not available");
      return BT::NodeStatus::FAILURE;
    };

    getInput<geometry_msgs::msg::PoseStamped>("target_pose", in_pose);

    std::thread subscriber_thread([node]() {
        rclcpp::spin_some(std::make_shared<Transform_calculation>());
    });
    subscriber_thread.detach();
    return BT::NodeStatus::RUNNING;
  }
   Transform_calculation::Transform_calculation() : Node("transform_calculation")
   {
    transform_point();
   }
   void Transform_calculation::transform_point()
  {
    tf2::Transform t_Map_BaseLink, t_UrBaseNew_BaseLink_tf2;
    tf2::Transform t_Map_UrBaseNewDesired;
    geometry_msgs::msg::TransformStamped t_UrBaseNew_BaseLink;
    tf2::Vector3 in_position = tf2::Vector3(in_pose.pose.position.x,
                                            in_pose.pose.position.y,
                                            in_pose.pose.position.z);

    t_Map_UrBaseNewDesired.setOrigin(in_position);

    tf2::Quaternion in_orientation = tf2::Quaternion(in_pose.pose.orientation.x,
                                                     in_pose.pose.orientation.y,
                                                     in_pose.pose.orientation.z,
                                                     in_pose.pose.orientation.w);

    t_Map_UrBaseNewDesired.setRotation(in_orientation);

    tf2::Vector3 tf_position = tf2::Vector3(-1.192,
                                           -0.041,
                                            0.128);
    t_UrBaseNew_BaseLink_tf2.setOrigin(tf_position);
    tf2::Quaternion tf_orientation = tf2::Quaternion(0.0,
                                                     0.0,
                                                     0.0,
                                                     1.0);
    t_UrBaseNew_BaseLink_tf2.setRotation(tf_orientation);
    t_Map_BaseLink.mult(t_Map_UrBaseNewDesired, t_UrBaseNew_BaseLink_tf2);
    tf2::Vector3 out_position = t_Map_BaseLink.getOrigin();
    tf2::Quaternion out_orientation = t_Map_BaseLink.getRotation();

    out_pose.pose.position.x = out_position.getX();
    out_pose.pose.position.y = out_position.getY();
    out_pose.pose.position.z = out_position.getZ();
    out_pose.pose.orientation.x = out_orientation.getX();
    out_pose.pose.orientation.y = out_orientation.getY();
    out_pose.pose.orientation.z = out_orientation.getZ();
    out_pose.pose.orientation.w = out_orientation.getW();

    RCLCPP_INFO(rclcpp::get_logger("Transform_pose"),
      "Input Position x: %f, y: %f, z: %f",
      in_pose.pose.position.x, in_pose.pose.position.y, in_pose.pose.position.z);
    RCLCPP_INFO(rclcpp::get_logger("Transform_pose"),
      "Input Orientation x: %f, y: %f, z: %f, w: %f",
      in_pose.pose.orientation.x, in_pose.pose.orientation.y,
      in_pose.pose.orientation.z, in_pose.pose.orientation.w);
    RCLCPP_INFO(rclcpp::get_logger("Transform_pose"), "Transformed Position x: %f, y: %f, z: %f",
      out_pose.pose.position.x, out_pose.pose.position.y, out_pose.pose.position.z);
    RCLCPP_INFO(rclcpp::get_logger("Transform_pose"),
      "Transformed Orientation x: %f, y: %f, z: %f, w: %f",
      out_pose.pose.orientation.x, out_pose.pose.orientation.y,
      out_pose.pose.orientation.z, out_pose.pose.orientation.w);
    check = true ;
  }

  BT::NodeStatus Transform_target_async::onRunning()
  {
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "In Transform_target_pose_onRunning");
    if (check == true)
    {
      setOutput("offset_target_pose", out_pose);
        return BT::NodeStatus::SUCCESS;
    } else{
        return BT::NodeStatus::RUNNING;
    }
  }

  void Transform_target_async::onHalted()
    {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Tansform_target_pose ABORTED");
    }

  }  // namespace rpm_behavior_tree

#include <string>
#include <memory>
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<rpm_behavior_tree::Transform_target_async>(
        name, config);
    };

  factory.registerBuilder<rpm_behavior_tree::Transform_target_async>(
    "Transform_pose", builder);
}
