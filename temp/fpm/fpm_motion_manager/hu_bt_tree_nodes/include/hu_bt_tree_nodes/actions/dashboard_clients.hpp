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



class RobotMode : public BT::AsyncActionNode
{
public:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<ur_dashboard_msgs::srv::GetRobotMode>::SharedPtr client_;
  std::shared_ptr<ur_dashboard_msgs::srv::GetRobotMode::Request> request_ = 
    std::make_shared<ur_dashboard_msgs::srv::GetRobotMode::Request>();
  rclcpp::Client<ur_dashboard_msgs::srv::GetRobotMode>::SharedFuture result_handle_;

  std::string node_name_ = "get_robot_mode_dashboard_client";
  std::string server_name_ = "/dashboard_client/get_robot_mode";

  RobotMode(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;
  void halt() override {}

  static BT::PortsList providedPorts(){ return {}; }
  
};

class Turnon : public BT::AsyncActionNode
{
public:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
  std::shared_ptr<std_srvs::srv::Trigger::Request> request_ = 
    std::make_shared<std_srvs::srv::Trigger::Request>();
  rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture result_handle_;

  std::string node_name_ = "turn_on_dashboard_client";
  std::string server_name_ = "/dashboard_client/power_on";

  Turnon(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;
  void halt() override {}

  static BT::PortsList providedPorts(){ return {}; }
  
};

class Turnoff : public BT::AsyncActionNode
{
public:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
  std::shared_ptr<std_srvs::srv::Trigger::Request> request_ = 
    std::make_shared<std_srvs::srv::Trigger::Request>();
  rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture result_handle_;

  std::string node_name_ = "turn_off_dashboard_client";
  std::string server_name_ = "/dashboard_client/power_off";

  Turnoff(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;
  void halt() override {}

  static BT::PortsList providedPorts(){ return {}; }
  
};


class Play : public BT::AsyncActionNode
{
public:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
  std::shared_ptr<std_srvs::srv::Trigger::Request> request_ = 
    std::make_shared<std_srvs::srv::Trigger::Request>();
  rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture result_handle_;

  std::string node_name_ = "play_prg_dashboard_client";
  std::string server_name_ = "/dashboard_client/play";

  Play(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;
  void halt() override {}

  static BT::PortsList providedPorts(){ return {}; }
};

class Pause : public BT::AsyncActionNode
{
public:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
  std::shared_ptr<std_srvs::srv::Trigger::Request> request_ = 
    std::make_shared<std_srvs::srv::Trigger::Request>();
  rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture result_handle_;

  std::string node_name_ = "pause_prg_dashboard_client";
  std::string server_name_ = "/dashboard_client/pause";

  Pause(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;
  void halt() override {}

  static BT::PortsList providedPorts(){ return {}; }
};

class Stop : public BT::AsyncActionNode
{
public:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
  std::shared_ptr<std_srvs::srv::Trigger::Request> request_ = 
    std::make_shared<std_srvs::srv::Trigger::Request>();
  rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture result_handle_;

  std::string node_name_ = "stop_prg_dashboard_client";
  std::string server_name_ = "/dashboard_client/stop";

  Stop(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;
  void halt() override {}

  static BT::PortsList providedPorts(){ return {}; }
};

class LoadProgram : public BT::AsyncActionNode
{
public:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<ur_dashboard_msgs::srv::Load>::SharedPtr client_;
  std::shared_ptr<ur_dashboard_msgs::srv::Load::Request> request_ = 
    std::make_shared<ur_dashboard_msgs::srv::Load::Request>();
  rclcpp::Client<ur_dashboard_msgs::srv::Load>::SharedFuture result_handle_;

  std::string node_name_ = "load_prg_dashboard_client";
  std::string server_name_ = "/dashboard_client/load_program";

  LoadProgram(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;
  void halt() override {}

  static BT::PortsList providedPorts(){ 
    return
    {
        BT::InputPort<std::string>("file_name")
    };
   }
};

class LoadInstallation : public BT::AsyncActionNode
{
public:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<ur_dashboard_msgs::srv::Load>::SharedPtr client_;
  std::shared_ptr<ur_dashboard_msgs::srv::Load::Request> request_ = 
    std::make_shared<ur_dashboard_msgs::srv::Load::Request>();
  rclcpp::Client<ur_dashboard_msgs::srv::Load>::SharedFuture result_handle_;

  std::string node_name_ = "load_inst_dashboard_client";
  std::string server_name_ = "/dashboard_client/load_installation";

  LoadInstallation(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;
  void halt() override {}

  static BT::PortsList providedPorts(){ 
    return
    {
        BT::InputPort<std::string>("file_name")
    };
   }
};


class GetLoadedProgram : public BT::AsyncActionNode
{
public:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<ur_dashboard_msgs::srv::GetLoadedProgram>::SharedPtr client_;
  std::shared_ptr<ur_dashboard_msgs::srv::GetLoadedProgram::Request> request_ = 
    std::make_shared<ur_dashboard_msgs::srv::GetLoadedProgram::Request>();
  rclcpp::Client<ur_dashboard_msgs::srv::GetLoadedProgram>::SharedFuture result_handle_;

  std::string node_name_ = "get_prg_dashboard_client";
  std::string server_name_ = "/dashboard_client/get_loaded_program";

  GetLoadedProgram(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;
  void halt() override {}

  static BT::PortsList providedPorts(){ 
    return
    {
    };
   }
};

class BrakeRelease : public BT::AsyncActionNode
{
public:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
  std::shared_ptr<std_srvs::srv::Trigger::Request> request_ = 
    std::make_shared<std_srvs::srv::Trigger::Request>();
  rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture result_handle_;

  std::string node_name_ = "release_brake_dashboard_client";
  std::string server_name_ = "/dashboard_client/brake_release";

  BrakeRelease(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;
  void halt() override {}

    static BT::PortsList providedPorts(){ 
    return
        {
        };
    }
};


class ProgramCondition : public BT::StatefulActionNode
{
public:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<ur_dashboard_msgs::srv::IsProgramRunning>::SharedPtr client_;
    std::shared_ptr<ur_dashboard_msgs::srv::IsProgramRunning::Request> request_ = 
      std::make_shared<ur_dashboard_msgs::srv::IsProgramRunning::Request>();
    rclcpp::Client<ur_dashboard_msgs::srv::IsProgramRunning>::SharedFuture result_handle_;
    
    std::string node_name_ = "prg_running_dashboard_client";
    std::string server_name_ = "/dashboard_client/program_running";
    
    ProgramCondition(
        const std::string & xml_tag_name,
        const BT::NodeConfiguration & conf);

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    static BT::PortsList providedPorts(){ 
        return
            {
            };
    }
};
