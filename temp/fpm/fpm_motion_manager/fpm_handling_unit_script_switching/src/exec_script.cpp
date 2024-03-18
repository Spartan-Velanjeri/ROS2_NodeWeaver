#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <unistd.h>
#include <chrono>
#include <thread>
using namespace std::chrono_literals; 

// exec_script.cpp
class ExecScript : public BT::AsyncActionNode
{
public:
    bool x;
    //rclcpp::Node::SharedPtr node_;
    explicit ExecScript(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts() { return {}; }
    
    
    BT::NodeStatus tick() override
    {
      
      
      std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("exec_script");
      rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client =
      node->create_client<std_srvs::srv::Trigger>("/io_and_status_controller/switch_script");
      while (!client->wait_for_service(1s)) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto result = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "script executed");
          return BT::NodeStatus::SUCCESS;
        } else {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to execute script");
          return BT::NodeStatus::FAILURE;
        }
    
    }
};