#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>


#include <chrono>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos_event.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <unistd.h>
#include <chrono>
#include <thread>
using namespace std::chrono_literals; 
class RemoveScene : public BT::AsyncActionNode
{
public:
    //rclcpp::Node::SharedPtr node_;
  explicit RemoveScene(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
    }

  static BT::PortsList providedPorts() { return {}; }
    
    
  BT::NodeStatus tick() override
    {
      
      
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("removescene");
    bool full_scene = true;
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    robot_model_loader::RobotModelLoader::Options opt;
    opt.robot_description_ = "robot_description";
    opt.load_kinematics_solvers_ = false;

    auto rml = std::make_shared<robot_model_loader::RobotModelLoader>(node, opt);
    planning_scene::PlanningScene ps(rml->getModel());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Remove the objects from the world");
    std::vector<std::string> object_ids;
    moveit_msgs::msg::CollisionObject collision_object;
    object_ids.push_back(collision_object.id);
    //object_ids.push_back(object_to_attach.id);
    planning_scene_interface.removeCollisionObjects(object_ids);
    return BT::NodeStatus::SUCCESS;
    }
};