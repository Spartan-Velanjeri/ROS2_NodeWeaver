#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <behaviortree_cpp_v3/action_node.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <yaml-cpp/yaml.h>
#include <algorithm>  // std::min
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <atomic>
#include <geometry_msgs/msg/pose.hpp>
#include <string>
#include <vector>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
using namespace std::chrono_literals;

class GetPoi : public BT::AsyncActionNode
{
public:
    rclcpp::Node::SharedPtr node_;
    explicit GetPoi(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
        std::shared_ptr<rclcpp::Node> node_ = rclcpp::Node::make_shared("load_points");
    }

    static BT::PortsList providedPorts() { return {BT::InputPort<int32_t>("total"), BT::InputPort<geometry_msgs::msg::PoseArray>("currentpoint"), BT::InputPort<int32_t>("point_index"), BT::OutputPort<geometry_msgs::msg::Pose>("drill_point")}; }
    
    
    BT::NodeStatus tick() override
    {
      int total;
      
      geometry_msgs::msg::PoseArray posesbb;
      geometry_msgs::msg::Pose p;
      int32_t i;
      //auto blackboard = BT::Blackboard::create();
      //total = blackboard->get("points_count_bb", total);
      getInput<int>("total",total);
      getInput<int>("point_index", i);
      getInput<geometry_msgs::msg::PoseArray>("currentpoint", posesbb);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "the total count is: %d ", total);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pose Found x: %f y: %f z: %f rx: %f ry: %f rz: %f rw: %f", posesbb.poses[i].position.x,
      posesbb.poses[i].position.y, posesbb.poses[i].position.z, posesbb.poses[i].orientation.x, posesbb.poses[i].orientation.y, posesbb.poses[i].orientation.z, posesbb.poses[i].orientation.w);
      p.position.x = posesbb.poses[i].position.x;
      p.position.y = posesbb.poses[i].position.y;
      p.position.z = posesbb.poses[i].position.z;
      p.orientation.x = posesbb.poses[i].orientation.x;
      p.orientation.y = posesbb.poses[i].orientation.y;
      p.orientation.z = posesbb.poses[i].orientation.z;
      p.orientation.w = posesbb.poses[i].orientation.w;
      setOutput("drill_point", p);
      return BT::NodeStatus::SUCCESS;
    }
    

};