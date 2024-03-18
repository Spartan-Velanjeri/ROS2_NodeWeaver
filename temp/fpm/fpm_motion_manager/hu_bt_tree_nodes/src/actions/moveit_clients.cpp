#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos_event.hpp>
#include <rclcpp/utilities.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <std_srvs/srv/trigger.hpp>
#include <unistd.h>
#include <chrono>
#include <thread>



using namespace std::chrono_literals;
struct Pose2D
  {
      double x, y, z, orientation_x, orientation_y, orientation_z;
  };
namespace BT{
  template <> inline
  Pose2D convertFromString(StringView key)
  {
      // three real numbers separated by semicolons
      auto parts = BT::splitString(key, ';');
      if (parts.size() != 6)
      {
          throw BT::RuntimeError("invalid input)");
      }
      else
      {
          Pose2D output;
          output.x     = convertFromString<double>(parts[0]);
          output.y     = convertFromString<double>(parts[1]);
          output.z = convertFromString<double>(parts[2]);
          output.orientation_x = convertFromString<double>(parts[3]);
          output.orientation_y = convertFromString<double>(parts[4]);
          output.orientation_z = convertFromString<double>(parts[5]);
    return output;
      }
    }  
}
struct Joints
{
    double base, shoulder, elbow, wrist1, wrist2, wrist3;
};
namespace BT{
template <> inline
Joints convertFromString(StringView key)
{
    // three real numbers separated by semicolons
    auto parts = BT::splitString(key, ';');
    if (parts.size() != 6)
    {
        throw BT::RuntimeError("invalid input)");
    }
    else
      {
          Joints output;
          output.base     = convertFromString<double>(parts[0]);
          output.shoulder     = convertFromString<double>(parts[1]);
          output.elbow = convertFromString<double>(parts[2]);
          output.wrist1 = convertFromString<double>(parts[3]);
          output.wrist2 = convertFromString<double>(parts[4]);
          output.wrist3 = convertFromString<double>(parts[5]);
      return output;
      }
    }
}

class LoadScene : public BT::AsyncActionNode
{
public:
  rclcpp::Node::SharedPtr node_;
  explicit LoadScene(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
    node_ = rclcpp::Node::make_shared("loadscene");
    }

  static BT::PortsList providedPorts() { return {BT::InputPort<std::string>("scene_file")}; }
    
    
  BT::NodeStatus tick() override
    {
       
    bool full_scene = true;
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_);

    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr pub_scene;
    rclcpp::Publisher<moveit_msgs::msg::PlanningSceneWorld>::SharedPtr pub_world_scene;

    if (full_scene)
      pub_scene = node_->create_publisher<moveit_msgs::msg::PlanningScene>(
          planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_TOPIC, 1);
    else
      pub_world_scene = node_->create_publisher<moveit_msgs::msg::PlanningSceneWorld>(
          planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC, 1);

    robot_model_loader::RobotModelLoader::Options opt;
    opt.robot_description_ = "robot_description";
    opt.load_kinematics_solvers_ = false;

    auto rml = std::make_shared<robot_model_loader::RobotModelLoader>(node_, opt);
    planning_scene::PlanningScene ps(rml->getModel());
    std::string file_name;
    auto input_handle = getInput<std::string>("scene_file",file_name);
    if (!input_handle.has_value()) {
      throw BT::RuntimeError("Missing required input [scene_file]: ",
                           input_handle.error());
    }
    std::ifstream f(file_name);
    if (ps.loadGeometryFromStream(f))
    {
      RCLCPP_INFO(node_->get_logger(), "Publishing geometry from '%s' ...", file_name.c_str());
      moveit_msgs::msg::PlanningScene ps_msg;
      ps.getPlanningSceneMsg(ps_msg);
      ps_msg.is_diff = true;

      unsigned int attempts = 0;
      while (pub_scene->get_subscription_count() < 1 && ++attempts < 100)
        rclcpp::sleep_for(500ms);

      if (full_scene)
        pub_scene->publish(ps_msg);
      else
        pub_world_scene->publish(ps_msg.world);
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::SUCCESS;
    
    }
};


class RemoveScene : public BT::AsyncActionNode
{
public:
  explicit RemoveScene(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
    }

  static BT::PortsList providedPorts() { return {}; }
    
    
  BT::NodeStatus tick() override
    {
      
      
    std::shared_ptr<rclcpp::Node> node_ = rclcpp::Node::make_shared("removescene");
    // bool full_scene = true;
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_);
    moveit::planning_interface::MoveGroupInterface move_group(node_, "ur_manipulator");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    robot_model_loader::RobotModelLoader::Options opt;
    opt.robot_description_ = "robot_description";
    opt.load_kinematics_solvers_ = false;

    auto rml = std::make_shared<robot_model_loader::RobotModelLoader>(node_, opt);
    planning_scene::PlanningScene ps(rml->getModel());
    RCLCPP_INFO(node_->get_logger(), "Remove the objects from the world");
    std::vector<std::string> object_ids;
    moveit_msgs::msg::CollisionObject collision_object;
    object_ids.push_back(collision_object.id);
    //object_ids.push_back(object_to_attach.id);
    planning_scene_interface.removeCollisionObjects(object_ids);
    return BT::NodeStatus::SUCCESS;
    }
};


class MoveitJoints : public BT::AsyncActionNode
{
public:

    //rclcpp::Node::SharedPtr node_;
    explicit MoveitJoints(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts() { return {BT::InputPort<Joints>("goal")}; }
    
    
    BT::NodeStatus tick() override
    {
        auto const node_ = std::make_shared<rclcpp::Node>(
        "execute_joint_goal",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
        );
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node_);
        std::thread([&executor]() { executor.spin(); }).detach();

  // Create a ROS logger
        auto const logger = node_->get_logger();

  // Create the MoveIt MoveGroup Interface
        using moveit::planning_interface::MoveGroupInterface;
        auto move_group = MoveGroupInterface(node_, "ur_manipulator");
        const moveit::core::JointModelGroup* joint_model_group =
            move_group.getCurrentState()->getJointModelGroup("ur_manipulator");
        Joints goal;
        auto input_handle = getInput<Joints>("goal",goal);
        if (!input_handle.has_value()) {
          throw BT::RuntimeError("Missing required input [goal]: ",
                         input_handle.error());
        }
        moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
  //
  // Next get the current set of joint values for the group.
        std::vector<double> joint_group_positions;
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
        RCLCPP_INFO(logger, "joint positions are %f %f %f %f %f %f", joint_group_positions[0], joint_group_positions[1], joint_group_positions[2], joint_group_positions[3],joint_group_positions[4], joint_group_positions[5]);
        // Now, let's modify one of the joints, plan to the new joint space goal, and visualize the plan.
        joint_group_positions[0] = -1.118;  // radians
        joint_group_positions[1] = 3.858;
        joint_group_positions[2] = -2.102;
        joint_group_positions[3] = -1.76;
        joint_group_positions[4] = -1.13;
        joint_group_positions[5] = 0.22371;
        bool within_bounds = move_group.setJointValueTarget(joint_group_positions);
        if (!within_bounds)
        {
            RCLCPP_INFO(logger, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
        }

        // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
        // The default values are 10% (0.1).
        // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
        // or set explicit factors in your code if you need your robot to move faster.
        move_group.setMaxVelocityScalingFactor(0.05);
        move_group.setMaxAccelerationScalingFactor(0.05);
        auto const [success, plan] = [&move_group]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group.plan(msg));
        return std::make_pair(ok, msg);
        }();
        
        // Execute the plan
        if(success) {
          move_group.execute(plan);
        } 
        else
        {
          RCLCPP_ERROR(logger, "Planing failed!");
          return BT::NodeStatus::FAILURE;
          executor.cancel();
        }
        executor.cancel();
        // Execute the plan
        return BT::NodeStatus::SUCCESS;
  // Shutdown ROS
    }
};

class MoveitControl : public BT::AsyncActionNode
{
public:
    bool x;
    std::shared_ptr<rclcpp::Node> node_; 
    explicit MoveitControl(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
      node_ = rclcpp::Node::make_shared("Moveit_control");
    }

    static BT::PortsList providedPorts() { return {}; }
    
    
    BT::NodeStatus tick() override
    {   
      rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client =
      node_->create_client<std_srvs::srv::Trigger>("/io_and_status_controller/resend_robot_program");
      while (!client->wait_for_service(1s)) {
        RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
      }
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto result = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node_, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_INFO(node_->get_logger(), "Shifted to External Control");
          return BT::NodeStatus::SUCCESS;
        } else {
          RCLCPP_ERROR(node_->get_logger(), "Failed to shift for external control");
          return BT::NodeStatus::FAILURE;
        }
      return BT::NodeStatus::SUCCESS;

    }
};

class MoveitEuler : public BT::AsyncActionNode
{
public:

  
    bool x;
    //rclcpp::Node::SharedPtr node_;
    explicit MoveitEuler(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts() { return {BT::InputPort<Pose2D>("goal")}; }
    
    
    BT::NodeStatus tick() override
    {
      
      
        //rclcpp::init(argc, argv);
        auto const node_ = std::make_shared<rclcpp::Node>(
        "moveit_absolute_euler_pose",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
        );

  // Create a ROS logger
        auto const logger = node_->get_logger();

  // Create the MoveIt MoveGroup Interface
        using moveit::planning_interface::MoveGroupInterface;
        auto move_group_interface = MoveGroupInterface(node_, "ur_manipulator");
        Pose2D goal;
        auto input_handle = getInput<Pose2D>("goal",goal);
        if (!input_handle.has_value()) {
          throw BT::RuntimeError("Missing required input [goal]: ",
            input_handle.error());
        }

        RCLCPP_INFO(logger, "Sending goal %f %f %f %f %f %f", goal.x, goal.y, goal.z, goal.orientation_x, goal.orientation_y, goal.orientation_z);
        // Set a target Pose
        geometry_msgs::msg::PoseStamped msg;
            //msg.orientation.w = 1.0;
        msg.pose.position.x = goal.x;
        msg.pose.position.y = goal.y;
        msg.pose.position.z = goal.z;
        msg.header.frame_id = "base_link";
        tf2::Quaternion myQuaternion;

        myQuaternion.setRPY(goal.orientation_x,goal.orientation_y,goal.orientation_z);

        //myQuaternion=myQuaternion.normalize();
        msg.pose.orientation.x = myQuaternion.getX();
        msg.pose.orientation.y = myQuaternion.getY();
        msg.pose.orientation.z = myQuaternion.getZ();
        msg.pose.orientation.w = myQuaternion.getW();
        //auto const target_pose = []{
            //return msg;
        //}();
        move_group_interface.setPoseReferenceFrame("base_link");
        RCLCPP_INFO(logger, "goal %f %f %f %f %f %f %f", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,msg.pose.orientation.w);
        RCLCPP_INFO(node_->get_logger(), "Reference frame: %s", move_group_interface.getPoseReferenceFrame().c_str());
        RCLCPP_INFO(node_->get_logger(), "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
        move_group_interface.setPoseTarget(msg);
        auto const [success, plan] = [&move_group_interface]{
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(move_group_interface.plan(msg));
            return std::make_pair(ok, msg);
        }();
        
        // Execute the plan
        if(success) {
        move_group_interface.execute(plan);
        } 
        else 
        {
            RCLCPP_ERROR(logger, "Planing failed!");
            return BT::NodeStatus::FAILURE;
        }
        RCLCPP_INFO(node_->get_logger(), "Reference frame: %s", move_group_interface.getPoseReferenceFrame().c_str());
        RCLCPP_INFO(node_->get_logger(), "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
        // Execute the plan
        return BT::NodeStatus::SUCCESS;
  // Shutdown ROS
    }
};

class MoveitCluster : public BT::AsyncActionNode
{
public:

    //rclcpp::Node::SharedPtr node_;
    explicit MoveitCluster(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts() { return {BT::InputPort<geometry_msgs::msg::Pose>("current_point")}; }
    
    
    BT::NodeStatus tick() override
    {
      
      
        //rclcpp::init(argc, argv);
        auto const node_ = std::make_shared<rclcpp::Node>(
        "moveit_absolute_pose",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
        );

  // Create a ROS logger
        auto const logger = node_->get_logger();

  // Create the MoveIt MoveGroup Interface
        using moveit::planning_interface::MoveGroupInterface;
        auto move_group_interface = MoveGroupInterface(node_, "ur_manipulator");
        Pose2D goal;
        geometry_msgs::msg::Pose current_point;
        
        auto input_handle = getInput<Pose2D>("goal",goal);
        if (!input_handle.has_value()) {
          throw BT::RuntimeError("Missing required input [goal]: ",
            input_handle.error());
        }
        input_handle = getInput<geometry_msgs::msg::Pose>("current_point",current_point);
        if (!input_handle.has_value()) {
          throw BT::RuntimeError("Missing required input [current_point]: ",
            input_handle.error());
        }
        
        RCLCPP_INFO(logger, "Sending goal %f %f %f", goal.x, goal.y, goal.z);
        // Set a target Pose
        geometry_msgs::msg::PoseStamped msg;
        msg.header.frame_id = "base_link_inertia";
        msg.pose.position.x = current_point.position.x;
        msg.pose.position.y = current_point.position.y;
        msg.pose.position.z = current_point.position.z;
        msg.pose.orientation.x = current_point.orientation.x;
        msg.pose.orientation.y = current_point.orientation.y;
        msg.pose.orientation.z = current_point.orientation.z;
        msg.pose.orientation.w = current_point.orientation.w;
        move_group_interface.setPoseTarget(msg);
        move_group_interface.setGoalTolerance(0.01);
        RCLCPP_INFO(logger, "goal %f %f %f %f %f %f %f", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,msg.pose.orientation.w);
        // Create a plan to that target pose
        auto const [success, plan] = [&move_group_interface]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
        }();
        
        // Execute the plan
        if(success) {
        move_group_interface.execute(plan);
        }
        else 
        {
          return BT::NodeStatus::FAILURE;
          RCLCPP_ERROR(logger, "Planing failed!");
        }
        
        return BT::NodeStatus::SUCCESS;
  // Shutdown ROS
    }
};


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<LoadScene>("loadscene");
  factory.registerNodeType<RemoveScene>("removescene");
  factory.registerNodeType<MoveitJoints>("moveitjoints");
  factory.registerNodeType<MoveitControl>("moveitcontrol");
  factory.registerNodeType<MoveitEuler>("moveit");
  factory.registerNodeType<MoveitCluster>("moveitcluster");
}