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
class LoadScene : public BT::AsyncActionNode
{
public:
    //rclcpp::Node::SharedPtr node_;
  explicit LoadScene(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
    }

  static BT::PortsList providedPorts() { return {BT::InputPort<std::string>("scene_file")}; }
    
    
  BT::NodeStatus tick() override
    {
       
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("loadscene");
    bool full_scene = true;
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr pub_scene;
    rclcpp::Publisher<moveit_msgs::msg::PlanningSceneWorld>::SharedPtr pub_world_scene;

    if (full_scene)
      pub_scene = node->create_publisher<moveit_msgs::msg::PlanningScene>(
          planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_TOPIC, 1);
    else
      pub_world_scene = node->create_publisher<moveit_msgs::msg::PlanningSceneWorld>(
          planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC, 1);

    robot_model_loader::RobotModelLoader::Options opt;
    opt.robot_description_ = "robot_description";
    opt.load_kinematics_solvers_ = false;

    auto rml = std::make_shared<robot_model_loader::RobotModelLoader>(node, opt);
    planning_scene::PlanningScene ps(rml->getModel());
    std::string file_name;
    getInput<std::string>("scene_file",file_name);
    std::ifstream f(file_name);
    if (ps.loadGeometryFromStream(f))
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing geometry from '%s' ...", "/home/avp1le/Documents/lab_scene.scene");
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
  
    
    }
};




