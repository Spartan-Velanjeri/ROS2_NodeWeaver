#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <behaviortree_cpp_v3/action_node.h>

#include <ur_dashboard_msgs/srv/get_robot_mode.hpp>
#include <ur_dashboard_msgs/srv/get_loaded_program.hpp>
#include <ur_dashboard_msgs/srv/load.hpp>
#include <ur_dashboard_msgs/srv/is_program_running.hpp>

#include <unistd.h>
#include <chrono>
#include <thread>

#include "hu_bt_tree_nodes/actions/dashboard_clients.hpp"

using namespace std::chrono_literals;

   
RobotMode::RobotMode(const std::string& name, const BT::NodeConfiguration& config)
    : BT::AsyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared(node_name_);
  //Parameter
  node_->declare_parameter("client_wait_time", 20);
  //Clients
  client_ = node_->create_client<ur_dashboard_msgs::srv::GetRobotMode>(server_name_);
}

BT::NodeStatus RobotMode::tick() {
    while (!client_->wait_for_service(std::chrono::seconds(
      node_->get_parameter("client_wait_time").as_int()))) {
    RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
  }
  
  result_handle_ = client_->async_send_request(request_);
    
  if (rclcpp::spin_until_future_complete(node_, result_handle_) ==
  rclcpp::FutureReturnCode::SUCCESS)
  {
    if (result_handle_.get()->robot_mode.mode == 3 || result_handle_.get()->robot_mode.mode == 5){
      RCLCPP_INFO(node_->get_logger(), "robot mode is %1d", result_handle_.get()->robot_mode.mode);
      RCLCPP_INFO(node_->get_logger(), "POWERING ON THE ROBOT");
      return BT::NodeStatus::FAILURE;
    } 
    else {
      return BT::NodeStatus::SUCCESS;
    }
  } 
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call service");
  }
  return BT::NodeStatus::SUCCESS;
}  

Turnon::Turnon(const std::string& name, const BT::NodeConfiguration& config)
    : BT::AsyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared(node_name_);
  //Parameter
  node_->declare_parameter("client_wait_time", 20);
  //Clients
  client_ = node_->create_client<std_srvs::srv::Trigger>(server_name_);
}

BT::NodeStatus Turnon::tick()
{
  
  while (!client_->wait_for_service(std::chrono::seconds(
      node_->get_parameter("client_wait_time").as_int()))) {
    RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
  }
  
  result_handle_ = client_->async_send_request(request_);
  if (rclcpp::spin_until_future_complete(node_, result_handle_) ==
    rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(node_->get_logger(), "power on robot");
      return BT::NodeStatus::FAILURE;
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to call service");
      return BT::NodeStatus::SUCCESS;
    }

}


Turnoff::Turnoff(const std::string& name, const BT::NodeConfiguration& config)
    : BT::AsyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared(node_name_);
  //Parameter
  node_->declare_parameter("client_wait_time", 20);
  //Clients
  client_ = node_->create_client<std_srvs::srv::Trigger>(server_name_);
}

BT::NodeStatus Turnoff::tick()
{

  while (!client_->wait_for_service(std::chrono::seconds(
      node_->get_parameter("client_wait_time").as_int()))) {
    RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
  }

  result_handle_ = client_->async_send_request(request_);
  if (rclcpp::spin_until_future_complete(node_, result_handle_) ==
    rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(node_->get_logger(), "power off robot");
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Failed to call service");
      return BT::NodeStatus::FAILURE;
    }

}


Play::Play(const std::string& name, const BT::NodeConfiguration& config)
      : BT::AsyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared(node_name_);
  //Parameter
  node_->declare_parameter("client_wait_time", 20);
  //Clients
  client_ = node_->create_client<std_srvs::srv::Trigger>(server_name_);
}

BT::NodeStatus Play::tick()
{

  while (!client_->wait_for_service(std::chrono::seconds(
      node_->get_parameter("client_wait_time").as_int()))) {
    RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
  }

  result_handle_ = client_->async_send_request(request_);
  if (rclcpp::spin_until_future_complete(node_, result_handle_) ==
    rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(node_->get_logger(), "playing the program");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Failed to call service");
      return BT::NodeStatus::FAILURE;
    }
}   



Pause::Pause(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
  {
  node_ = rclcpp::Node::make_shared(node_name_);
  //Parameter
  node_->declare_parameter("client_wait_time", 20);
  //Clients
  client_ = node_->create_client<std_srvs::srv::Trigger>(server_name_);
  }

BT::NodeStatus Pause::tick()
{
  while (!client_->wait_for_service(std::chrono::seconds(
      node_->get_parameter("client_wait_time").as_int()))) {
    RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
  }
  
  result_handle_ = client_->async_send_request(request_);

  if (rclcpp::spin_until_future_complete(node_, result_handle_) ==
    rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(node_->get_logger(), "pausing the program");
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Failed to call service");
      return BT::NodeStatus::FAILURE;
    }
}   


Stop::Stop(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared(node_name_);
  //Parameter
  node_->declare_parameter("client_wait_time", 20);
  //Clients
  client_ = node_->create_client<std_srvs::srv::Trigger>(server_name_);
}

BT::NodeStatus Stop::tick() 
{
  
  while (!client_->wait_for_service(std::chrono::seconds(
    node_->get_parameter("client_wait_time").as_int()))) {
    RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
  }
  
  result_handle_ = client_->async_send_request(request_);
  if (rclcpp::spin_until_future_complete(node_, result_handle_) ==
    rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(node_->get_logger(), "stopping the program");
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Failed to call service");
      return BT::NodeStatus::FAILURE;
    }

}   


LoadProgram::LoadProgram(const std::string& name, const BT::NodeConfiguration& config)
    : BT::AsyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared(node_name_);
  //Parameter
  node_->declare_parameter("client_wait_time", 20);
  //Clients
  client_ = node_->create_client<ur_dashboard_msgs::srv::Load>(server_name_);  
}

BT::NodeStatus LoadProgram::tick() {
  //Read inputs and handle input errors
  auto input_handle = getInput<std::string>("file_name",request_->filename);
  if (!input_handle.has_value()) {
      throw BT::RuntimeError("Missing required input [filename]: ",
                             input_handle.error());
  }
  
  while (!client_->wait_for_service(std::chrono::seconds(
    node_->get_parameter("client_wait_time").as_int()))) {
    RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
  }
  
  result_handle_ = client_->async_send_request(request_); 
  if (rclcpp::spin_until_future_complete(node_, result_handle_) ==
  rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(node_->get_logger(), "%s", result_handle_.get()->answer.c_str());
    return BT::NodeStatus::SUCCESS;
  }  
  else {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call service");
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}   
      

LoadInstallation::LoadInstallation(const std::string& name, const BT::NodeConfiguration& config)
    : BT::AsyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared(node_name_);
  //Parameter
  node_->declare_parameter("client_wait_time", 20);
  //Clients
  client_ = node_->create_client<ur_dashboard_msgs::srv::Load>(server_name_);
}

BT::NodeStatus LoadInstallation::tick() {
  //Read inputs and handle input errors
  auto input_handle = getInput<std::string>("file_name",request_->filename);
  if (!input_handle.has_value()) {
      throw BT::RuntimeError("Missing required input [filename]: ",
                             input_handle.error());
  }
  //Wait for service to be come available
  while (!client_->wait_for_service(std::chrono::seconds(
      node_->get_parameter("client_wait_time").as_int()))) {
    RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
  }
  result_handle_ = client_->async_send_request(request_);
  
  if (rclcpp::spin_until_future_complete(node_, result_handle_) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(node_->get_logger(), "%s", result_handle_.get()->answer.c_str());
    return BT::NodeStatus::SUCCESS;
  }   
  else {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call service");
    return BT::NodeStatus::FAILURE;
  }
return BT::NodeStatus::SUCCESS;
}  
      

GetLoadedProgram::GetLoadedProgram(const std::string& name, const BT::NodeConfiguration& config)
    : BT::AsyncActionNode(name, config)
{
    node_ = rclcpp::Node::make_shared(node_name_);
    //Parameter
    node_->declare_parameter("client_wait_time", 20);
    //Clients
    client_ = node_->create_client<ur_dashboard_msgs::srv::GetLoadedProgram>(server_name_);
}

BT::NodeStatus GetLoadedProgram::tick() 
{
  while (!client_->wait_for_service(std::chrono::seconds(
      node_->get_parameter("client_wait_time").as_int()))) {
    RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
  }
  result_handle_ = client_->async_send_request(request_);

  if (rclcpp::spin_until_future_complete(node_, result_handle_) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(node_->get_logger(), "the Program loaded is %s", result_handle_.get()->program_name.c_str());
    return BT::NodeStatus::SUCCESS;
  }   
  else 
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call service");
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}    
      

BrakeRelease::BrakeRelease(const std::string& name, const BT::NodeConfiguration& config)
    : BT::AsyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared(node_name_);
  //Parameter
  node_->declare_parameter("client_wait_time", 20);
  //Clients
  client_ = node_->create_client<std_srvs::srv::Trigger>(server_name_);
}

BT::NodeStatus BrakeRelease::tick()
{
  while (!client_->wait_for_service(std::chrono::seconds(
      node_->get_parameter("client_wait_time").as_int()))) {
    RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
  }
  result_handle_ = client_->async_send_request(request_);
  if (rclcpp::spin_until_future_complete(node_, result_handle_) ==
    rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(node_->get_logger(), "Releasing the brakes");
      return BT::NodeStatus::SUCCESS;
    } 
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to call service");
      return BT::NodeStatus::FAILURE;
    }
}


ProgramCondition::ProgramCondition(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared(node_name_);
  //Parameter
  node_->declare_parameter("client_wait_time", 1);
  //Clients
  client_ = node_->create_client<ur_dashboard_msgs::srv::IsProgramRunning>(server_name_);    }

BT::NodeStatus ProgramCondition::onStart(){
    RCLCPP_INFO(node_->get_logger(), "started program condition check");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ProgramCondition::onRunning() {
  while (!client_->wait_for_service(std::chrono::seconds(
    node_->get_parameter("client_wait_time").as_int()))) {
      RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
  }
  result_handle_ = client_->async_send_request(request_);
    
  if (rclcpp::spin_until_future_complete(node_, result_handle_) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      if (result_handle_.get()->program_running == false){ //checking whether Robot program is running or not
        return BT::NodeStatus::SUCCESS;
      }
      else {
        return BT::NodeStatus::RUNNING;
      }
    } 
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to call service");
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}
void ProgramCondition::onHalted(){
    RCLCPP_INFO(node_->get_logger(), "program condition check is halted");
}


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<RobotMode>("isrobotrunning");
  factory.registerNodeType<Turnon>("turnon");
  factory.registerNodeType<Turnoff>("turnoff");
  factory.registerNodeType<Play>("play");
  factory.registerNodeType<Pause>("pause");
  factory.registerNodeType<Stop>("stop");
  factory.registerNodeType<LoadProgram>("loadur");
  factory.registerNodeType<LoadInstallation>("loadinstallation");
  factory.registerNodeType<GetLoadedProgram>("getloadedprogram");
  factory.registerNodeType<BrakeRelease>("brakerelease");
  factory.registerNodeType<ProgramCondition>("programcondition");

}
