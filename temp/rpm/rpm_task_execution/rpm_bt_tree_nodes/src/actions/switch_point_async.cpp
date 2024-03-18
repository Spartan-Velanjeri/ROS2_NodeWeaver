//  Copyright 2024 Robert Bosch GmbH and its subsidiaries
//
//  All rights reserved, also regarding any disposal, exploitation, reproduction,
//  editing, distribution, as well as in the event of applications for industrial
//  property rights.
#include "rpm_bt_tree_nodes/actions/switch_point_async.hpp"


namespace rpm_behavior_tree
{ bool check;
  geometry_msgs::msg::PoseStamped in_pose, out_pose;
  BT::NodeStatus Switch_point_async::onStart()
  {
    rclcpp::Node::SharedPtr node_switch = rclcpp::Node::make_shared("calculation_switch_point");
    check = false;
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Start switch point Tick!!!");
     if (!node_switch)
    {
       RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "node not available");
      return BT::NodeStatus::FAILURE;
    };
     auto tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_switch->get_clock());
     auto tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    getInput<geometry_msgs::msg::PoseStamped>("target_pose", in_pose);
    std::thread subscriber_thread([node_switch]() {
        rclcpp::spin_some(std::make_shared<Calculation>());
    });
    subscriber_thread.detach();
    return BT::NodeStatus::RUNNING;
  }
   Calculation::Calculation() : Node("calculation")
   {
    get_point();
   }
   void Calculation::get_point()
  {
    float distance_to_target = -0.2;
    geometry_msgs::msg::TransformStamped t1 = geometry_msgs::msg::TransformStamped();
    geometry_msgs::msg::TransformStamped t_UrBaseNew_BaseLink;

    // switch pose in target pose frame

    tf2::Transform t_targetFrame_switchPoseFrame;
    t_targetFrame_switchPoseFrame.setOrigin(tf2::Vector3(distance_to_target, 0.0, 0.0));
    t_targetFrame_switchPoseFrame.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

    // target frame in pks frame
    tf2::Transform t_map_targetFrame;
    tf2::Vector3 in_position = tf2::Vector3(in_pose.pose.position.x,
                                            in_pose.pose.position.y,
                                            in_pose.pose.position.z);
    t_map_targetFrame.setOrigin(in_position);
    tf2::Quaternion in_orientation = tf2::Quaternion(in_pose.pose.orientation.x,
                                                     in_pose.pose.orientation.y,
                                                     in_pose.pose.orientation.z,
                                                     in_pose.pose.orientation.w);
    t_map_targetFrame.setRotation(in_orientation);

    // calculation of switch pose frame in map
    tf2::Transform t_map_switchPoseFrame;
    t_map_switchPoseFrame.mult(t_map_targetFrame, t_targetFrame_switchPoseFrame);

    // get position and orientation of switch pose
    tf2::Vector3 out_position = t_map_switchPoseFrame.getOrigin();
    tf2::Quaternion out_orientation = t_map_switchPoseFrame.getRotation();

    // set output to switch pose
    out_pose.header.frame_id = "map";
    out_pose.pose.position.x = out_position.getX();
    out_pose.pose.position.y = out_position.getY();
    out_pose.pose.position.z = out_position.getZ();
    out_pose.pose.orientation.x = out_orientation.getX();
    out_pose.pose.orientation.y = out_orientation.getY();
    out_pose.pose.orientation.z = out_orientation.getZ();
    out_pose.pose.orientation.w = out_orientation.getW();

    RCLCPP_INFO(rclcpp::get_logger("switch_point"),
      "Input Position x: %f, y: %f, z: %f",
      in_pose.pose.position.x, in_pose.pose.position.y, in_pose.pose.position.z);
    RCLCPP_INFO(rclcpp::get_logger("switch_point"),
      "Input Orientation x: %f, y: %f, z: %f, w: %f",
      in_pose.pose.orientation.x, in_pose.pose.orientation.y,
      in_pose.pose.orientation.z, in_pose.pose.orientation.w);
    RCLCPP_INFO(rclcpp::get_logger("switch_point"),
      "Switch Position x: %f, y: %f, z: %f",
      out_pose.pose.position.x, out_pose.pose.position.y, out_pose.pose.position.z);
    RCLCPP_INFO(rclcpp::get_logger("switch_point"),
      "Switch Orientation x: %f, y: %f, z: %f, w: %f",
      out_pose.pose.orientation.x, out_pose.pose.orientation.y,
      out_pose.pose.orientation.z, out_pose.pose.orientation.w);
    check = true ;
  }

  BT::NodeStatus Switch_point_async::onRunning()
  {
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "In Switch_point_onRunning");
    if (check == true)
    {
      setOutput("pose_switch_point", out_pose);
        return BT::NodeStatus::SUCCESS;
    } else{
        return BT::NodeStatus::RUNNING;
    }
  }

  void Switch_point_async::onHalted()
    {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Switch_point ABORTED");
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
      return std::make_unique<rpm_behavior_tree::Switch_point_async>(
        name, config);
    };

  factory.registerBuilder<rpm_behavior_tree::Switch_point_async>(
    "Switch_point", builder);
}
