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
struct Pose2D
{
    double x, y, z;
};

namespace BT
{
template <> inline
Pose2D convertFromString(StringView key)
{
    // three real numbers separated by semicolons
    auto parts = BT::splitString(key, ';');
    if (parts.size() != 3)
    {
        throw BT::RuntimeError("invalid input)");
    }
    else
    {
        Pose2D output;
        output.x     = convertFromString<double>(parts[0]);
        output.y     = convertFromString<double>(parts[1]);
        output.z = convertFromString<double>(parts[2]);
        //output.w = convertFromString<double>(parts[3]);
        //output.r_x = convertFromString<double>(parts[3]);
        //output.r_y = convertFromString<double>(parts[4]);
        //output.r_z = convertFromString<double>(parts[5]);
	return output;
    }
}
} 
class Moveit : public BT::AsyncActionNode
{
public:
    //rclcpp::Node::SharedPtr node_;
    explicit Moveit(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts() { return {BT::InputPort<Pose2D>("goal")}; }
    
    
    BT::NodeStatus tick() override
    {
      
      
        //rclcpp::init(argc, argv);
        auto const node = std::make_shared<rclcpp::Node>(
        "hello_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
        );

  // Create a ROS logger
        auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
        using moveit::planning_interface::MoveGroupInterface;
        auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
        Pose2D goal;
        getInput<Pose2D>("goal",goal);
        RCLCPP_INFO(logger, "Sending goal %f %f %f", goal.x, goal.y, goal.z);
        // Set a target Pose
        geometry_msgs::msg::PoseStamped msg;
        msg.header.frame_id = "base_link_inertia";
        msg.pose.position.x = goal.x;
        msg.pose.position.y = goal.y;
        msg.pose.position.z = goal.z;
        //msg.pose.orientation.x = -0.006040;
        //msg.pose.orientation.y = -0.705872;
        //msg.pose.orientation.z = 0.708309;
        //msg.pose.orientation.w = 0.002389;
        msg.pose.orientation.x = -0.000013;
        msg.pose.orientation.y = -0.707803;
        msg.pose.orientation.z = -0.706409;
        msg.pose.orientation.w = 0.001111;
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
        } else {
        RCLCPP_ERROR(logger, "Planing failed!");
        }
        // Execute the plan
        return BT::NodeStatus::SUCCESS;
  // Shutdown ROS
    }
};