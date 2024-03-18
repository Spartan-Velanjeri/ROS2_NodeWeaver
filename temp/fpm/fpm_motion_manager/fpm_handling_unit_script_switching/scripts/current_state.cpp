#include <memory>
#include "tf2_msgs/msg/tf_message.hpp"
#include <std_msgs/msg/float32.hpp>
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <std_srvs/srv/trigger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
using namespace std::chrono_literals;
class CurrentState : public rclcpp::Node
{
  public:
    CurrentState()
    : Node("current_state")
    {   
        
      target_frame_ = this->declare_parameter<std::string>("target_frame", "wrist_3_link");
      //initial_frame_ = this->declare_parameter<std::string>("initial_frame", "wrist_3_link");
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        // Create a ROS logger
        
      timer_ = this->create_wall_timer(
          500ms, std::bind(&CurrentState::timer_callback, this));
        // Create the MoveIt MoveGroup Interface
        
    }
    void timer_callback()
    {   
        std::string fromFrameRel = target_frame_.c_str();
        std::string toFrameRel = "base_link_inertia";
        geometry_msgs::msg::TransformStamped t;
        t = tf_buffer_->lookupTransform(
              toFrameRel, fromFrameRel,
              tf2::TimePointZero);
        RCLCPP_INFO(this->get_logger(), "x: '%.6f'", t.transform.translation.x);
        RCLCPP_INFO(this->get_logger(), "y: '%.6f'", t.transform.translation.y);
        RCLCPP_INFO(this->get_logger(), "z: '%.6f'", t.transform.translation.z);
        RCLCPP_INFO(this->get_logger(), "rx: '%.6f'", t.transform.rotation.x);
        RCLCPP_INFO(this->get_logger(), "ry: '%.6f'", t.transform.rotation.y);
        RCLCPP_INFO(this->get_logger(), "rz: '%.6f'", t.transform.rotation.z);
        RCLCPP_INFO(this->get_logger(), "rw: '%.6f'", t.transform.rotation.w);
        auto const node = std::make_shared<rclcpp::Node>(
        "hello_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
        );
        auto const logger = rclcpp::get_logger("hello_moveit");
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        std::thread([&executor]() { executor.spin(); }).detach();
        using moveit::planning_interface::MoveGroupInterface;
        auto move_group = MoveGroupInterface(node, "ur_manipulator");
        const moveit::core::JointModelGroup* joint_model_group =
                    move_group.getCurrentState()->getJointModelGroup("ur_manipulator");
        moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
        //
        // Next get the current set of joint values for the group.
        std::vector<double> joint_group_positions, samples;
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
        RCLCPP_INFO(logger, "joint positions in radians are %f %f %f %f %f %f", joint_group_positions[0], joint_group_positions[1], joint_group_positions[2], 
                joint_group_positions[3],joint_group_positions[4], joint_group_positions[5]);
        // Shutdown ROS
        executor.cancel();
        
        
    }
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string target_frame_;
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
};
int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CurrentState>());
  rclcpp::shutdown();
  return 0;
}