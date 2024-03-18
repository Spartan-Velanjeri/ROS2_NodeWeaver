#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
struct Joints
{
    double base, shoulder, elbow, wrist1, wrist2, wrist3;
};

namespace BT
{
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
      
      
        //rclcpp::init(argc, argv);
        auto const node = std::make_shared<rclcpp::Node>(
        "hello_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
        );
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        std::thread([&executor]() { executor.spin(); }).detach();

  // Create a ROS logger
        auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
        using moveit::planning_interface::MoveGroupInterface;
        auto move_group = MoveGroupInterface(node, "ur_manipulator");
        const moveit::core::JointModelGroup* joint_model_group =
            move_group.getCurrentState()->getJointModelGroup("ur_manipulator");
        Joints goal;
        getInput<Joints>("goal",goal);
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
        } else {
        RCLCPP_ERROR(logger, "Planing failed!");
        }
        executor.cancel();
        // Execute the plan
        return BT::NodeStatus::SUCCESS;
  // Shutdown ROS
    }
};