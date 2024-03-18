#include <functional>
#include <memory>
#include <thread>
//Actions, services, messages
#include "bautiro_ros_interfaces/action/set_handling_unit_configured_pose.hpp"
#include "bautiro_ros_interfaces/action/move_handling_unit_relativ_to_workplane.hpp"
#include "bautiro_ros_interfaces/action/move_lift_absolute.hpp"
#include "bautiro_ros_interfaces/action/move_handling_unit_absolute.hpp"
//Ros topics
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include "tf2_ros/buffer.h"
#include <tf2/exceptions.h>
#include <tf2/convert.h>
#include <geometry_msgs/msg/transform.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>


//ROS includes
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/parameter.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "handling_unit_motion_manager/visibility_control.h"

const std::string MOVE_GROUP = "hu_ur_6axis";

namespace fpm_motion_manager
{
class FPMActionServer : public rclcpp::Node
{
public:
  using ConfiguredPose = bautiro_ros_interfaces::action::SetHandlingUnitConfiguredPose;
  using GoalHandleConfiguredPose = rclcpp_action::ServerGoalHandle<ConfiguredPose>;

  using MoveRelative = bautiro_ros_interfaces::action::MoveHandlingUnitRelativToWorkplane;
  using GoalHandleMoveRelative = rclcpp_action::ServerGoalHandle<MoveRelative>;

  using MoveAbsolute = bautiro_ros_interfaces::action::MoveHandlingUnitAbsolute;
  using GoalHandleMoveAbsolute = rclcpp_action::ServerGoalHandle<MoveAbsolute>;



  /// Move group interface for the robot
  moveit::planning_interface::MoveGroupInterface move_group_;

  MOTION_MANAGER_CPP_PUBLIC
  explicit FPMActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("fpm_cu_action_server", options),
    move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), MOVE_GROUP)
  {
    using namespace std::placeholders;

    this->action_server_configured_pose_ = rclcpp_action::create_server<ConfiguredPose>(
      this,
      "fpm_set_hu_configured_pose",
      std::bind(&FPMActionServer::handle_goal_configured_pose, this, _1, _2),
      std::bind(&FPMActionServer::handle_cancel_configured_pose, this, _1),
      std::bind(&FPMActionServer::handle_accepted_configured_pose, this, _1));

    this->action_server_move_relative_ = rclcpp_action::create_server<MoveRelative>(
      this,
      "fpm_set_move_relative",
      std::bind(&FPMActionServer::handle_goal_move_relative, this, _1, _2),
      std::bind(&FPMActionServer::handle_cancel_move_relative, this, _1),
      std::bind(&FPMActionServer::handle_accepted_move_relative, this, _1));

    this->action_server_move_absolute_ = rclcpp_action::create_server<MoveAbsolute>(
      this,
      "fpm_set_move_hu_absolute",
      std::bind(&FPMActionServer::handle_goal_move_absolute, this, _1, _2),
      std::bind(&FPMActionServer::handle_cancel_move_absolute, this, _1),
      std::bind(&FPMActionServer::handle_accepted_move_absolute, this, _1));

    // Declare and read parameters
    this->declare_parameter("configured_pose_names");
    //Configured tcp pose
    std::vector<std::string> configured_pose_name = this-> get_parameter("configured_pose_names").as_string_array();
    
    for (auto param_name : configured_pose_name){
      RCLCPP_INFO(this->get_logger(), "Configured pose: %s", param_name.c_str());
      //Declare and get parameters
      this->declare_parameter(param_name + ".id");
      this->declare_parameter(param_name + ".position");
      this->declare_parameter(param_name + ".orientation");

      int id = this->get_parameter(param_name + ".id").as_int();
      std::vector<double> position = this->get_parameter(param_name + ".position").as_double_array();
      std::vector<double> orientation = this->get_parameter(param_name + ".orientation").as_double_array();
      
      //Store configured poses into map 
      position.insert(position.end(),orientation.begin(),orientation.end()); //concetenate arrays
      this->configured_poses[id] = position;
      RCLCPP_INFO(this->get_logger(), "Configured pose: %i", id);
      
    };

    // Declare and read joint pose parameters
    this->declare_parameter("configured_joint_pose_names");
    //Configured joint pose
    std::vector<std::string> configured_joint_pose_names = this-> get_parameter("configured_joint_pose_names").as_string_array();

    for (auto param_name : configured_joint_pose_names){
      RCLCPP_INFO(this->get_logger(), "Configured joint pose: %s", param_name.c_str());
      //Declare and get parameters
      this->declare_parameter(param_name + ".id");
      this->declare_parameter(param_name + ".joints");
      int id = this->get_parameter(param_name + ".id").as_int();
      std::vector<double> joint_position = this->get_parameter(param_name + ".joints").as_double_array();
      
      //Store configured poses into map 

      this->configured_joint_poses[id] = joint_position;
      RCLCPP_INFO(this->get_logger(), "Configured joint pose: %i", id);
      
    };

    // Initialize tf listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    this->move_group_.setMaxVelocityScalingFactor(0.01);

    RCLCPP_INFO(this->get_logger(),"Planning frame %s", move_group_.getPlanningFrame().c_str());
    RCLCPP_INFO(this->get_logger(),"End effector link %s", move_group_.getEndEffectorLink().c_str());
    RCLCPP_INFO(this->get_logger(),"Reference frame %s", move_group_.getPoseReferenceFrame().c_str());
    RCLCPP_INFO(this->get_logger(), "Server ready");
  }

private:
  //Action server
  rclcpp_action::Server<ConfiguredPose>::SharedPtr action_server_configured_pose_;
  rclcpp_action::Server<MoveRelative>::SharedPtr action_server_move_relative_;
  rclcpp_action::Server<MoveAbsolute>::SharedPtr action_server_move_absolute_; 


  //Publisher

  //TF listener for lift movement  
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;


  // map for storing the configured values
  std::map<unsigned int, std::vector<double>>  configured_poses;
  std::map<unsigned int, std::vector<double>>  configured_joint_poses;
  
  //Action server for moving the handling unit to configured positions


  rclcpp_action::GoalResponse handle_goal_configured_pose(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ConfiguredPose::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received configured pose request number %d", goal->handling_unit_configured_pose);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel_configured_pose(
    const std::shared_ptr<GoalHandleConfiguredPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    this->move_group_.stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted_configured_pose(const std::shared_ptr<GoalHandleConfiguredPose> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&FPMActionServer::execute_configured_pose, this, _1), goal_handle}.detach();
  }

  void execute_configured_pose(const std::shared_ptr<GoalHandleConfiguredPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ConfiguredPose::Feedback>();
    auto & fb_progress_normalized = feedback->progress_normalized;
    auto result = std::make_shared<ConfiguredPose::Result>();
    std::map<unsigned int, std::vector<double>>::iterator it;
    std::vector<double> joint_group_positions, joint_group_target_positions;

    it = this->configured_joint_poses.find(goal->handling_unit_configured_pose);
    
    RCLCPP_INFO(this->get_logger(), "move to configured joint pose id %i ", goal->handling_unit_configured_pose);
    if (it == configured_poses.end())
    {
      RCLCPP_INFO(this->get_logger(), "Configured joint pose id %i not found", goal->handling_unit_configured_pose);
      //TODO cancel action
      //move_group_.stop()
    }
    else
    {
    //Get desired joint pose   
    joint_group_positions = it->second;

    // Plan and execute motion
    this->move_group_.setJointValueTarget(joint_group_positions);
    this->move_group_.getJointValueTarget(joint_group_target_positions);
    for (double jointvalue : joint_group_target_positions)
      RCLCPP_INFO(this->get_logger(), "joint target %f",jointvalue);
    this->move_group_.move();
    
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->response_code = fb_progress_normalized;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

// Action server for moving the handling unit relative on workplane

    rclcpp_action::GoalResponse handle_goal_move_relative(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveRelative::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received relative move command  x:%f  y:%f  z:%f"  , goal->relative_target_position[0],goal->relative_target_position[1], goal->relative_target_position[2]);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel_move_relative(
    const std::shared_ptr<GoalHandleMoveRelative> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    this->move_group_.stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted_move_relative(const std::shared_ptr<GoalHandleMoveRelative> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    RCLCPP_INFO(this->get_logger(), "Spin new thread for move relative action");
    std::thread{std::bind(&FPMActionServer::execute_move_relative, this, _1), goal_handle}.detach();
  }

  void execute_move_relative(const std::shared_ptr<GoalHandleMoveRelative> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Moving Relative");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveRelative::Feedback>();
    auto & fb_progress_normalized = feedback->progress_normalized; //TODO generate feedback using current waypoint 
    auto result = std::make_shared<MoveRelative::Result>();
    std::vector<float> delta_position = goal->relative_target_position;
    
    geometry_msgs::msg::PoseStamped desired_pose, start_pose;

    if (!delta_position.empty()){
      RCLCPP_INFO(this->get_logger(),"pose dimension %li ", sizeof(delta_position)/sizeof(delta_position[0]));
    }
    else
    {
      RCLCPP_INFO(this->get_logger(),"pose dimension is empty");
      //TODO cancel action
    }


    start_pose = this->move_group_.getCurrentPose("");
    
    RCLCPP_INFO(this->get_logger(),"move relative to x=%f y=%f z=%f",delta_position[0], delta_position[1], delta_position[2]);

    desired_pose = start_pose;
    desired_pose.pose.position.x +=  delta_position[0];   
    desired_pose.pose.position.y +=  delta_position[1];
    desired_pose.pose.position.z +=  delta_position[2];
    RCLCPP_INFO(this->get_logger(),"move trans to x=%f y=%f z=%f",
                desired_pose.pose.position.x,
                desired_pose.pose.position.y, 
                desired_pose.pose.position.z);

    RCLCPP_INFO(this->get_logger(),"move orient to x=%f y=%f z=%f w=%f",
                desired_pose.pose.orientation.x, 
                desired_pose.pose.orientation.y, 
                desired_pose.pose.orientation.z, 
                desired_pose.pose.orientation.w);
    
    // Plan and execute motion
    this->move_group_.setPoseTarget(desired_pose.pose);
    moveit::planning_interface::MoveGroupInterface::Plan absolute_plan;
    auto success = this->move_group_.plan(absolute_plan);// == moveit::core::MoveItErrorCode::SUCCESS);
    //RCLCPP_INFO(this->get_logger(), "jnt %s", success ? "TRUE" : "FAILED");
    
    std::vector<double> joint_group_current_positions, joint_group_target_positions;
    //Get current joint poses
    joint_group_current_positions = this->move_group_.getCurrentJointValues();
    
    RCLCPP_INFO(this->get_logger(), "Current joint position");
    for (double jointvalue : joint_group_current_positions)
      RCLCPP_INFO(this->get_logger(), "jnt curr %f",jointvalue);


    //Get target joint poses
    this->move_group_.getJointValueTarget(joint_group_target_positions);

    for (double jointvalue : joint_group_target_positions)
      RCLCPP_INFO(this->get_logger(), "jnt target %f",jointvalue);
      
    this->move_group_.execute(absolute_plan);
    
    
    // Check if goal is done
    if (rclcpp::ok()) {
      result->current_position.push_back(desired_pose.pose.position.x);
      result->current_position.push_back(desired_pose.pose.position.y);
      result->response_code = fb_progress_normalized;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }


  rclcpp_action::GoalResponse handle_goal_move_absolute(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveAbsolute::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received absolute move command  x:%f  y:%f",
      goal->absolute_target_position.pose.position.x,
      goal->absolute_target_position.pose.position.y);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel_move_absolute(
    const std::shared_ptr<GoalHandleMoveAbsolute> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    this->move_group_.stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted_move_absolute(const std::shared_ptr<GoalHandleMoveAbsolute> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    RCLCPP_INFO(this->get_logger(), "Spin new thread for move absolute action");
    std::thread{std::bind(&FPMActionServer::execute_move_absolute, this, _1), goal_handle}.detach();
  }

  void execute_move_absolute(const std::shared_ptr<GoalHandleMoveAbsolute> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Moving Absolute");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<MoveAbsolute::Result>();
  
    geometry_msgs::msg::PoseStamped current_pose, target_pose, target_pose_tf;
    geometry_msgs::msg::TransformStamped transformation;
    std::string fromFrameRel, toFrameRel;
    
    current_pose = this->move_group_.getCurrentPose("");
    target_pose = goal->absolute_target_position;

    RCLCPP_INFO(this->get_logger(),"current_pose frame_id %s ",
                current_pose.header.frame_id.c_str());

    RCLCPP_INFO(this->get_logger(),"current_pose to x=%f y=%f z=%f",
                current_pose.pose.position.x,
                current_pose.pose.position.y, 
                current_pose.pose.position.z);

    RCLCPP_INFO(this->get_logger(),"current_pose to x=%f y=%f z=%f w=%f",
                current_pose.pose.orientation.x, 
                current_pose.pose.orientation.y, 
                current_pose.pose.orientation.z, 
                current_pose.pose.orientation.w);

    fromFrameRel = target_pose.header.frame_id;
    toFrameRel = current_pose.header.frame_id;
            
    bool flag = true;
    while(flag)
    {
      try {
        transformation = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
        RCLCPP_INFO(this->get_logger(), "Transformation found");
        flag = false;
      } 
      catch (const tf2::TransformException & ex) {
        flag = true;
        RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
      }
    }

    // Simple pose transformation 
    RCLCPP_INFO(this->get_logger(),"Transformig pose to %s ", toFrameRel.c_str());
    tf2::doTransform(target_pose,target_pose_tf,transformation);

    target_pose_tf.pose.orientation = current_pose.pose.orientation;

    RCLCPP_INFO(this->get_logger(),"target_pose_tf frame_id %s ",
                target_pose_tf.header.frame_id.c_str());

    RCLCPP_INFO(this->get_logger(),"target_pose_tf to x=%f y=%f z=%f",
                target_pose_tf.pose.position.x,
                target_pose_tf.pose.position.y, 
                target_pose_tf.pose.position.z);

    RCLCPP_INFO(this->get_logger(),"target_pose_tf to x=%f y=%f z=%f w=%f",
                target_pose_tf.pose.orientation.x, 
                target_pose_tf.pose.orientation.y, 
                target_pose_tf.pose.orientation.z, 
                target_pose_tf.pose.orientation.w);

    this->move_group_.setPoseReferenceFrame(target_pose_tf.header.frame_id.c_str());
    this->move_group_.setPoseTarget(target_pose_tf.pose);
    this->move_group_.move();

    // Check if goal is done
    if (rclcpp::ok()) {
      result->current_position = this->move_group_.getCurrentPose();
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }



};  // class FPMActionServer

}  // namespace fpm_cu
RCLCPP_COMPONENTS_REGISTER_NODE(fpm_motion_manager::FPMActionServer)
