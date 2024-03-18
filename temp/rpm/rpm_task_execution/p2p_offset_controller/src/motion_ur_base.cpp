//  Copyright 2023, 2024 Robert Bosch GmbH and its subsidiaries
//
//  All rights reserved, also regarding any disposal, exploitation, reproduction,
//  editing, distribution, as well as in the event of applications for industrial
//  property rights.
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>
#include <memory>
#include <cmath>
#include <string>
#include "p2p_offset_controller/dynamics.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "bautiro_ros_interfaces/action/move_ur_base.hpp"

namespace p2p_offset_controller
{
class OffsetUrBaseController : public rclcpp::Node
{
public:
  state state_;
  cartesian_param cartesian_param_;
  twist_vel twist_vel_;
  double wheel_radius_ = 0.21;
  tf2::Matrix3x3 m;
  tf2::Quaternion q;
  geometry_msgs::msg::PoseStamped p;
  geometry_msgs::msg::Pose target;
  rclcpp_action::Server<bautiro_ros_interfaces::action::MoveUrBase>::SharedPtr action_server_;
  geometry_msgs::msg::TransformStamped t;
  geometry_msgs::msg::TransformStamped current_pose;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  size_t count_;

  double controller_gain_p = 1;
  float distance_threshold = 0.05;

  // maximum velocities
  float max_rot_vel = 0.5;
  float max_lin_vel = 0.5;


  OffsetUrBaseController()
  : Node("offset_ur_base_controller"), count_(0)
  {
    using namespace std::placeholders;
    this->action_server_ = rclcpp_action::create_server<
      bautiro_ros_interfaces::action::MoveUrBase>(
      this, "move_ur_base",
      std::bind(&OffsetUrBaseController::handle_goal, this, _1, _2),
      std::bind(&OffsetUrBaseController::handle_cancel, this, _1),
      std::bind(&OffsetUrBaseController::handle_accepted, this, _1));
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/rpm_velocity_controller/cmd_vel_unstamped", 10);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const bautiro_ros_interfaces::action::MoveUrBase::Goal> goal)
  {
    RCLCPP_INFO(
      this->get_logger(), "Received goal request with x: %f, y: %f",
      goal->target_pose.position.x, goal->target_pose.position.y);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
      bautiro_ros_interfaces::action::MoveUrBase>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
      bautiro_ros_interfaces::action::MoveUrBase>> goal_handle)
  {
    //  this needs to return quickly to avoid blocking the executor, so spin up a new thread
    using namespace std::placeholders;
    std::thread{std::bind(&OffsetUrBaseController::execute, this, _1), goal_handle}.detach();
  }

  void execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
      bautiro_ros_interfaces::action::MoveUrBase>> goal_handle)
  {
    // RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto message = geometry_msgs::msg::Twist();

    state_.wheel_radius = wheel_radius_;
    auto success = std_msgs::msg::Bool();
    success.data = false;
    rclcpp::Rate loop_rate(10);

    state_.wheel_separation = 0.57;
    std::string fromFrameRel;
    std::string toFrameRel;
    // std::string fromFrameRel = "left_base";
    // std::string toFrameRel = "base_link";
    // try {
    //   t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
    //   state_.wheel_separation = 2*abs(t.transform.translation.y);
    // } catch (const tf2::TransformException & ex) {
    //   RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
    //                    toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
    //   return;
    // }

    state_.offset_x = 1.192000;
    state_.offset_y = 0.041000;
    RCLCPP_INFO(
      this->get_logger(), "Offset_x = '%f',  Offset_y = '%f'",
      state_.offset_x, state_.offset_y);
    // fromFrameRel = "rpm_working_space_center_point";
    // toFrameRel = "base_link";
    // try {
    //   t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
    //   state_.offset_x = t.transform.translation.x;
    //   state_.offset_y = t.transform.translation.y;
    // } catch (const tf2::TransformException & ex) {
    //   RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
    //                    toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
    //   return;
    // }

    // get pose error in PKS
    fromFrameRel = "rpm_working_space_center_point";
    toFrameRel = "map";
    try {
      t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
      p.pose.position.x = goal->target_pose.position.x - t.transform.translation.x;
      p.pose.position.y = goal->target_pose.position.y - t.transform.translation.y;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(
        this->get_logger(), "Could not transform %s to %s: %s",
        toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
      return;
    }
    // rotate robot frame to correct orientation
    fromFrameRel = "map";
    // this frame has position of handling_unit_base and orientation of base_link
    toFrameRel = "rpm_working_space_center_point";
    try {
      t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
      //  set position of transform to 0
      t.transform.translation.x = 0.0;
      t.transform.translation.y = 0.0;
      t.transform.translation.z = 0.0;
      p.pose.orientation.x = t.transform.rotation.x;
      p.pose.orientation.y = t.transform.rotation.y;
      p.pose.orientation.z = t.transform.rotation.z;
      p.pose.orientation.w = t.transform.rotation.w;
      tf2::doTransform(p, p, t);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(
        this->get_logger(), "Could not transform %s to %s: %s",
        toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
      return;
    }

    state_.wheel_separation = 0.57;
    // fromFrameRel = "left_base";
    // toFrameRel = "base_link";
    // try {
    //   t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
    //   state_.wheel_separation = 2*abs(t.transform.translation.y);
    // } catch (const tf2::TransformException & ex) {
    //   RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
    //                      toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
    //   return;
    // }
    cartesian_param_.dot_x = p.pose.position.x;
    cartesian_param_.dot_y = p.pose.position.y;

    // Controller loop until the pose error is smaller then distance threshold
    while (sqrt(
        cartesian_param_.dot_x * cartesian_param_.dot_x +
        cartesian_param_.dot_y * cartesian_param_.dot_y) > distance_threshold)
    {
      // get robot geometry: trackwidth
      state_.wheel_separation = 0.570000;
      RCLCPP_INFO(this->get_logger(), "wheel_separation = '%f'", state_.wheel_separation);

      // std::string fromFrameRel = "left_base";
      // std::string toFrameRel = "base_link";
      // try {
      //   t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
      //   state_.wheel_separation = 2*abs(t.transform.translation.y);
      //   RCLCPP_INFO(this->get_logger(), "wheel_separation = '%f'", state_.wheel_separation);
      // } catch (const tf2::TransformException & ex) {
      //   RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",toFrameRel.c_str(),
      //                                                fromFrameRel.c_str(), ex.what());
      //   return;
      // }
      // get robot geometry: offset of center of rotation and reference frame
      // [INFO] [1700237602.826685183] [offset_ur_base_controller]: wheel_separ│
      // ation = '0.570000'                                                    │
      // [INFO] [1700237602.826769664] [offset_ur_base_controller]: Offset_x = │
      // '1.192000',  Offset_y = '0.041000'
      state_.offset_x = 1.192000;
      state_.offset_y = 0.041000;
      RCLCPP_INFO(
        this->get_logger(), "Offset_x = '%f',  Offset_y = '%f'",
        state_.offset_x, state_.offset_y);
      // fromFrameRel = "rpm_working_space_center_point";
      // toFrameRel = "base_link";
      // try {
      //   t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
      //   state_.offset_x = t.transform.translation.x;
      //   state_.offset_y = t.transform.translation.y;
      //   RCLCPP_INFO(this->get_logger(), "Offset_x = '%f',
      //            Offset_y = '%f'", state_.offset_x, state_.offset_y);
      // } catch (const tf2::TransformException & ex) {
      //   RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
      //              toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
      //   return;
      // }
      // get pose error in PKS
      fromFrameRel = "rpm_working_space_center_point";
      toFrameRel = "map";
      try {
        current_pose = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
        p.pose.position.x = goal->target_pose.position.x - current_pose.transform.translation.x;
        p.pose.position.y = goal->target_pose.position.y - current_pose.transform.translation.y;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
          this->get_logger(), "Could not transform %s to %s: %s",
          toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
        return;
      }

      // rotate robot frame to correct orientation
      fromFrameRel = "map";
      toFrameRel = "rpm_working_space_center_point";
      try {
        t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);

        // set position of transform to 0
        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;

        p.pose.orientation.x = t.transform.rotation.x;
        p.pose.orientation.y = t.transform.rotation.y;
        p.pose.orientation.z = t.transform.rotation.z;
        p.pose.orientation.w = t.transform.rotation.w;
        tf2::doTransform(p, p, t);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
          this->get_logger(), "Could not transform %s to %s: %s",
          toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
        return;
      }

      state_.wheel_separation = 0.570000;

      // This is the actual controller !!
      cartesian_param_.dot_x = p.pose.position.x * controller_gain_p;
      cartesian_param_.dot_y = p.pose.position.y * controller_gain_p;
      inv_offset_dynamics(cartesian_param_, state_, twist_vel_);

      message.linear.x = twist_vel_.v_x;
      message.angular.z = twist_vel_.w_z;

      // saturate velocities
      float sv = (twist_vel_.v_x > 0) - (twist_vel_.v_x < 0);
      float sw = (twist_vel_.w_z > 0) - (twist_vel_.w_z < 0);
      if (abs(twist_vel_.v_x) > 0.5 && abs(twist_vel_.w_z) < abs(twist_vel_.v_x)) {
        message.angular.z = sw * abs(twist_vel_.w_z / twist_vel_.v_x) * 0.5;
        message.linear.x = sv * 0.5;
      } else if (abs(twist_vel_.w_z) > 0.5 && abs(twist_vel_.w_z) > abs(twist_vel_.v_x)) {
        message.angular.z = sw * 0.5;
        message.linear.x = sv * abs(twist_vel_.v_x / twist_vel_.w_z) * 0.5;
      }

      RCLCPP_DEBUG(
        this->get_logger(),
        "Transformed error: ex = '%f', ey = '%f'",
        cartesian_param_.dot_x, cartesian_param_.dot_y);

      RCLCPP_DEBUG(
        this->get_logger(),
        "Target position: x = '%f', y='%f' ",
        goal->target_pose.position.x,
        goal->target_pose.position.y);

      RCLCPP_DEBUG(
        this->get_logger(),
        "Current position: x = '%f', y='%f' ",
        current_pose.transform.translation.x,
        current_pose.transform.translation.y);

      RCLCPP_DEBUG(
        this->get_logger(),
        "Residual Error: e = '%f' ",
        sqrt(
          cartesian_param_.dot_x * cartesian_param_.dot_x +
          cartesian_param_.dot_y * cartesian_param_.dot_y));

      RCLCPP_DEBUG(
        this->get_logger(),
        "Publishing: v = '%f', w = '%f' ",
        message.linear.x, message.angular.z);

      RCLCPP_DEBUG(
        this->get_logger(),
        "=============================================================");
      publisher_->publish(message);
      loop_rate.sleep();
    }

    auto result = std::make_shared<
      bautiro_ros_interfaces::action::MoveUrBase::Result>();


    message.linear.x = 0.0;
    message.angular.z = 0.0;
    publisher_->publish(message);

    success.data = true;
    // Check if goal is done
    if (rclcpp::ok()) {
      result->pose_reached = success;
      goal_handle->succeed(result);
      // RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};  //  OffsetUrBaseController class end
}  //  namespace p2p_offset_controller


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<
      p2p_offset_controller::OffsetUrBaseController>());
  rclcpp::shutdown();
  return 0;
}
