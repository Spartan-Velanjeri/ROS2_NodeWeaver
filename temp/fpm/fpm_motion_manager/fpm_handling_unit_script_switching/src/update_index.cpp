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

class UpdatePoi : public BT::AsyncActionNode
{
public:
    rclcpp::Node::SharedPtr node_;
    explicit UpdatePoi(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
        std::shared_ptr<rclcpp::Node> node_ = rclcpp::Node::make_shared("load_points");
    }

    static BT::PortsList providedPorts() { return {BT::InputPort<int32_t>("total"), BT::InputPort<int32_t>("point_index"), BT::OutputPort<int32_t>("update_index")}; }
    
    
    BT::NodeStatus tick() override
    {
      int total;
      int32_t i;
      //auto blackboard = BT::Blackboard::create();
      //total = blackboard->get("points_count_bb", total);
      getInput<int>("total",total);
      getInput<int>("point_index", i);
      i = i+1;
      setOutput("update_index", i);
      //blackboard->set("point_index", i);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "the current index is: %d ", i);
      
      return BT::NodeStatus::SUCCESS;
    }
    

};