//#include <BT_ros2/src/on_robot.hpp>
//#include "on_robot.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <behaviortree_cpp_v3/action_node.h>
#include <unistd.h>
#include <chrono>
#include <thread>
using namespace std::chrono_literals;

class Turnon : public BT::AsyncActionNode
{
public:
    //rclcpp::Node::SharedPtr node_;
    explicit Turnon(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts() { return {}; }
    
    
    BT::NodeStatus tick() override
    {
      
      
      std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("Turn_on_the_robot");
      rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client =
      node->create_client<std_srvs::srv::Trigger>("/dashboard_client/power_on");
      while (!client->wait_for_service(1s)) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto result = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "power on robot");
          return BT::NodeStatus::FAILURE;
        } else {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
          return BT::NodeStatus::SUCCESS;
        }
    
    }
};

class Turnoff : public BT::AsyncActionNode
{
public:
    //rclcpp::Node::SharedPtr node_;
    explicit Turnoff(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts() { return {}; }
    
    
    BT::NodeStatus tick() override
    {
      
      
      std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("Turn_off_the_robot");
      rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client =
      node->create_client<std_srvs::srv::Trigger>("/dashboard_client/power_off");
      while (!client->wait_for_service(1s)) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto result = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "power off robot");
          return BT::NodeStatus::SUCCESS;
        } else {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
          return BT::NodeStatus::FAILURE;
        }
    
    }
};

class Play : public BT::AsyncActionNode
{
public:
    //rclcpp::Node::SharedPtr node_;
    explicit Play(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
    }
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override
    {
      std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("play_the_program");
      rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client =
      node->create_client<std_srvs::srv::Trigger>("/dashboard_client/play");
      while (!client->wait_for_service(1s)) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto result = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "playing the program");
          return BT::NodeStatus::SUCCESS;
        } else {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
          return BT::NodeStatus::FAILURE;
        }
    
    }   
};

class Pause : public BT::AsyncActionNode
{
public:
    //rclcpp::Node::SharedPtr node_;
    explicit Pause(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
    }
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override
    {
      std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("pause_the_program");
      rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client =
      node->create_client<std_srvs::srv::Trigger>("/dashboard_client/pause");
      while (!client->wait_for_service(1s)) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto result = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pausing the program");
          return BT::NodeStatus::SUCCESS;
        } else {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
          return BT::NodeStatus::FAILURE;
        }
    
    }   
};

class Stop : public BT::AsyncActionNode
{
public:
    //rclcpp::Node::SharedPtr node_;
    explicit Stop(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
    }
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override
    {
      std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("stop_the_program");
      rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client =
      node->create_client<std_srvs::srv::Trigger>("/dashboard_client/stop");
      while (!client->wait_for_service(1s)) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto result = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "stopping the program");
          return BT::NodeStatus::RUNNING;
        } else {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
          return BT::NodeStatus::FAILURE;
        }
    
    }   
};

class Brakerelease : public BT::AsyncActionNode
{
public:
    //rclcpp::Node::SharedPtr node_;
    explicit Brakerelease(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts() { return {}; }
    
    
    BT::NodeStatus tick() override
    {
      
      
      std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("releasing_the_brakes");
      rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client =
      node->create_client<std_srvs::srv::Trigger>("/dashboard_client/brake_release");
      while (!client->wait_for_service(1s)) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto result = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Releasing the brakes");
          return BT::NodeStatus::FAILURE;
        } else {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
          return BT::NodeStatus::FAILURE;
        }
    
    }
      
    

};