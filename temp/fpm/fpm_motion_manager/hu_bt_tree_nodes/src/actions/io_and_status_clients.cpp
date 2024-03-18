#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp_v3/action_node.h>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "ur_msgs/msg/io_states.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "ur_msgs/msg/tool_data_msg.hpp"
#include "ur_msgs/srv/set_auxiliary_script_arguments.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <unistd.h>
#include <chrono>
#include <thread>
using namespace std::chrono_literals;

class LoadScript : public BT::AsyncActionNode
{
public:
    bool x;
    std::shared_ptr<rclcpp::Node> node_;

    explicit LoadScript(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
      node_ = rclcpp::Node::make_shared("load_script");
    }

    static BT::PortsList providedPorts() { return {}; }
        
    BT::NodeStatus tick() override
    {
      rclcpp::Client<ur_msgs::srv::SetAuxiliaryScriptArguments>::SharedPtr client =
      node_->create_client<ur_msgs::srv::SetAuxiliaryScriptArguments>("/io_and_status_controller/set_aux_script_arguments");
      auto request = std::make_shared<ur_msgs::srv::SetAuxiliaryScriptArguments::Request>();
      request->drill_pos_x_replace = -0.2;
      request->drill_pos_y_replace = -0.55;
      request->drill_pos_z_replace = -0.20;
      request->drill_depth_replace = 0.1;
      while (!client->wait_for_service(1s)) {
        RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
      }
      //int emptyarray[4];
      //request->all_arguments = emptyarray;
      auto result = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node_, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_INFO(node_->get_logger(), "Script loaded");
          return BT::NodeStatus::SUCCESS;
        } else {
          RCLCPP_ERROR(node_->get_logger(), "failed to load script");
          return BT::NodeStatus::FAILURE;
        }
    
    }
};

// exec_script.cpp
class ExecScript : public BT::AsyncActionNode
{
public:
    bool x;
    rclcpp::Node::SharedPtr node_;

    explicit ExecScript(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
      node_ = rclcpp::Node::make_shared("exec_script");
    }

    static BT::PortsList providedPorts() { return {}; }
    
    
    BT::NodeStatus tick() override
    {      
      rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client =
      node_->create_client<std_srvs::srv::Trigger>("/io_and_status_controller/switch_script");
      while (!client->wait_for_service(1s)) {
        RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
      }
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto result = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node_, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_INFO(node_->get_logger(), "script executed");
          return BT::NodeStatus::SUCCESS;
        } else {
          RCLCPP_ERROR(node_->get_logger(), "Failed to execute script");
          return BT::NodeStatus::FAILURE;
        }
    
    }
};
// tool_force.cpp


class Toolforce : public BT::AsyncActionNode
{
    public:
        
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_;
        Toolforce(const std::string& name, const BT::NodeConfiguration& config)
            : BT::AsyncActionNode(name, config)
        {
            node_ = rclcpp::Node::make_shared("tool_forces");
            sub_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>("/force_torque_sensor_broadcaster/wrench", 1000, std::bind(&Toolforce::toolforceCallback,this, std::placeholders::_1));
        }

        void toolforceCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
        {
            RCLCPP_INFO(node_->get_logger(), "the tool force is: ");
            RCLCPP_INFO(node_->get_logger(), "x: %.2f", msg.get()->wrench.force.x); 
            RCLCPP_INFO(node_->get_logger(), "y: %.2f", msg.get()->wrench.force.y);
            RCLCPP_INFO(node_->get_logger(), "z: %.2f", msg.get()->wrench.force.z);
            RCLCPP_INFO(node_->get_logger(), "the tool torque is: ");
            RCLCPP_INFO(node_->get_logger(), "x: %.2f", msg.get()->wrench.torque.x); 
            RCLCPP_INFO(node_->get_logger(), "y: %.2f", msg.get()->wrench.torque.y);
            RCLCPP_INFO(node_->get_logger(), "z: %.2f", msg.get()->wrench.torque.z);
        }
        static BT::PortsList providedPorts() { return {}; }

        virtual BT::NodeStatus tick() override
        {
            rclcpp::spin_some(node_);
            
            //RCLCPP_INFO(node_->get_logger(), "the value is %1d", interrupt_event);
            return BT::NodeStatus::SUCCESS; 
            
        }

        
};
// Resolve Redundunt Classes and callbacks
// iocondition.cpp

        

class Iostatecondition : public BT::AsyncActionNode
{   
    public:
        int output;
        
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<ur_msgs::msg::IOStates>::SharedPtr subs_;
        Iostatecondition(const std::string& name, const BT::NodeConfiguration& config)
            : BT::AsyncActionNode(name, config)
        {
            node_ = rclcpp::Node::make_shared("io_condition");
            subs_ = node_->create_subscription<ur_msgs::msg::IOStates>("/io_and_status_controller/io_states", 1000, std::bind(&Iostatecondition::iostatesCallback, this, std::placeholders::_1));
        }
        void iostatesCallback(const ur_msgs::msg::IOStates::SharedPtr msg)
        {    
            RCLCPP_INFO(node_->get_logger(), "digital_outputs");
            int i;
            for (i=0;i<8;i++){
                RCLCPP_INFO(node_->get_logger(), "pin: %1d", msg.get()->digital_out_states[i].pin);
                RCLCPP_INFO(node_->get_logger(), "the output state is %1d", msg.get()->digital_out_states[i].state);
            }
            output = msg.get()->digital_out_states[4].state; // fpm_handling_unit_bt
        }
        static BT::PortsList providedPorts() { return {}; }
        
        BT::NodeStatus tick() override
        {
            rclcpp::spin_some(node_);
            if (output == 1){
                return BT::NodeStatus::FAILURE;
            }
            else {
                return BT::NodeStatus::SUCCESS;
            }
            
        }

     

        
};
// topic_iostates.cpp

class Iostates : public BT::AsyncActionNode
{
    public:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<ur_msgs::msg::IOStates>::SharedPtr sub_;

        void iostateCallback(const ur_msgs::msg::IOStates::SharedPtr msg)
        {
            RCLCPP_INFO(node_->get_logger(), "digital_outputs");
            int i;
            for (i=0;i<8;i++){
                RCLCPP_INFO(node_->get_logger(), "pin: %1d", msg.get()->digital_out_states[i].pin);
                RCLCPP_INFO(node_->get_logger(), "the output state is %1d", msg.get()->digital_out_states[i].state);
            }
        }
        Iostates(const std::string& name, const BT::NodeConfiguration& config)
            : BT::AsyncActionNode(name, config)
        {
            node_ = rclcpp::Node::make_shared("io_states");
            sub_ = node_->create_subscription<ur_msgs::msg::IOStates>("/io_and_status_controller/io_states", 1000, std::bind(&Iostates::iostateCallback, this, std::placeholders::_1));
        }

        static BT::PortsList providedPorts() { return {}; }

        virtual BT::NodeStatus tick() override
        {
            rclcpp::spin_some(node_);
            
            //RCLCPP_INFO(node_->get_logger(), "the value is %1d", interrupt_event);
            return BT::NodeStatus::SUCCESS; 
            
        }

        
};

// topic_iostates.cpp




class Jointstates : public BT::AsyncActionNode
{
    public:
       
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
        Jointstates(const std::string& name, const BT::NodeConfiguration& config)
            : BT::AsyncActionNode(name, config)
        {
            node_ = rclcpp::Node::make_shared("joint_states");
            sub_ = node_->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 1000, std::bind(&Jointstates::jointstateCallback, this, std::placeholders::_1));
        }
         void jointstateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            RCLCPP_INFO(node_->get_logger(), "joint states are: ");
            RCLCPP_INFO(node_->get_logger(), "Base: %.2f", msg.get()->position[0]); 
            RCLCPP_INFO(node_->get_logger(), "Shoulder: %.2f", msg.get()->position[1]);
            RCLCPP_INFO(node_->get_logger(), "Elbow: %.2f", msg.get()->position[2]);
            RCLCPP_INFO(node_->get_logger(), "Wrist1: %.2f", msg.get()->position[3]); 
            RCLCPP_INFO(node_->get_logger(), "Wrist2: %.2f", msg.get()->position[4]);
            RCLCPP_INFO(node_->get_logger(), "Wrist3: %.2f", msg.get()->position[5]);
            
        }
        static BT::PortsList providedPorts() { return {}; }
        
        virtual BT::NodeStatus tick() override
        {
            rclcpp::spin_some(node_);
            
            //RCLCPP_INFO(node_->get_logger(), "the value is %1d", interrupt_event);
            return BT::NodeStatus::SUCCESS; 
            
        }

        
};

// topic_tooltemp.cpp


class Tooltemp : public BT::AsyncActionNode
{
    public:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<ur_msgs::msg::ToolDataMsg>::SharedPtr sub_;

        void interruptCallback(const ur_msgs::msg::ToolDataMsg::SharedPtr msg)
        {
            RCLCPP_INFO(node_->get_logger(), "the tool temperature is %f", msg.get()->tool_temperature);
            RCLCPP_INFO(node_->get_logger(), "the tool mode is %1d", msg.get()->tool_mode);
            RCLCPP_INFO(node_->get_logger(), "the tool current is %1f", msg.get()->tool_current);
        }
        Tooltemp(const std::string& name, const BT::NodeConfiguration& config)
            : BT::AsyncActionNode(name, config)
        {
            node_ = rclcpp::Node::make_shared("tool_data");
            sub_ = node_->create_subscription<ur_msgs::msg::ToolDataMsg>("/io_and_status_controller/tool_data", 1000, std::bind(&Tooltemp::interruptCallback, this, std::placeholders::_1));
        }

        static BT::PortsList providedPorts() { return {}; }

        virtual BT::NodeStatus tick() override
        {
            rclcpp::spin_some(node_);
            
            //RCLCPP_INFO(node_->get_logger(), "the value is %1d", interrupt_event);
            return BT::NodeStatus::SUCCESS; 
            
        }

        
};

class IOCondition : public BT::StatefulActionNode
{
  public:
    int output;    
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<ur_msgs::msg::IOStates>::SharedPtr subs_;
    explicit IOCondition(const std::string& name, const BT::NodeConfiguration& config)
      : BT::StatefulActionNode(name, config)
    {
    	node_ = rclcpp::Node::make_shared("io_condition_states");
        subs_ = node_->create_subscription<ur_msgs::msg::IOStates>("/io_and_status_controller/io_states", 1000, std::bind(&IOCondition::ioCallback, this, std::placeholders::_1));
    }
    void ioCallback(const ur_msgs::msg::IOStates::SharedPtr msg)
        {    
            RCLCPP_INFO(node_->get_logger(), "digital_outputs");
            int i;
            for (i=0;i<8;i++){
                RCLCPP_INFO(node_->get_logger(), "pin: %1d", msg.get()->digital_out_states[i].pin);
                RCLCPP_INFO(node_->get_logger(), "the output state is %1d", msg.get()->digital_out_states[i].state);
            }
            output = msg.get()->digital_out_states[4].state; // fpm_handling_unit_bt
            // output = msg.get()->digital_out_states[3].state; // fpm_handling_unit_script_switching
        }
    static BT::PortsList providedPorts()
    {
        return{};
    }

    
    BT::NodeStatus onStart() override{
        RCLCPP_INFO(node_->get_logger(), "started io condition check");
        return BT::NodeStatus::RUNNING;
    }

    
    BT::NodeStatus onRunning() override{
        rclcpp::spin_some(node_);
        RCLCPP_INFO(node_->get_logger(), "digital_outputs");
        if (output == 1){
            return BT::NodeStatus::SUCCESS;
        }
        else {
            return BT::NodeStatus::RUNNING;
        }
        return BT::NodeStatus::RUNNING;
    }

    
    void onHalted() override{
        RCLCPP_INFO(node_->get_logger(), "io condition check is halted");
    }
};


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<LoadScript>("loadscript");
  factory.registerNodeType<ExecScript>("execscript");
  factory.registerNodeType<Toolforce>("toolforce");
  factory.registerNodeType<Iostatecondition>("iostatecondition");
  factory.registerNodeType<Iostates>("iostates");
  factory.registerNodeType<Jointstates>("jointstates");
  factory.registerNodeType<Tooltemp>("tooltemp");
  factory.registerNodeType<IOCondition>("iocondition");
}