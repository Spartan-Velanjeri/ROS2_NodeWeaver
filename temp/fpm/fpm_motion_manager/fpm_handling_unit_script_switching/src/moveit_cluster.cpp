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
        geometry_msgs::msg::Pose current_point;
        getInput<Pose2D>("goal",goal);
        getInput<geometry_msgs::msg::Pose>("current_point",current_point);
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
        } else {
        RCLCPP_ERROR(logger, "Planing failed!");
        }
        // Execute the plan
        return BT::NodeStatus::SUCCESS;
  // Shutdown ROS
    }
};