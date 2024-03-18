#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp_v3/action_node.h>
#include <std_msgs/msg/float32.hpp>
#include "ur_msgs/msg/io_states.hpp"
#include <unistd.h>
#include <chrono>
#include <thread>
using namespace std::chrono_literals;
int output;

int iostatesCallback(const ur_msgs::msg::IOStates::SharedPtr msg)
{
    
    output = msg.get()->digital_out_states[3].state;
}

class Iostatecondition : public BT::AsyncActionNode
{
    public:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<ur_msgs::msg::IOStates>::SharedPtr subs_;
        Iostatecondition(const std::string& name, const BT::NodeConfiguration& config)
            : BT::AsyncActionNode(name, config)
        {
            node_ = rclcpp::Node::make_shared("io_states");
            subs_ = node_->create_subscription<ur_msgs::msg::IOStates>("/io_and_status_controller/io_states", 1000, iostatesCallback);
        }

        static BT::PortsList providedPorts() { return {}; }
        
        BT::NodeStatus tick() override
        {
            rclcpp::spin_some(node_);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "digital_outputs");
            //BT::NodeStatus::FAILURE;
            if (output == 1){
                return BT::NodeStatus::FAILURE;
            }
            else {
                return BT::NodeStatus::SUCCESS;
            }
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "the value is %1d", interrupt_event); 
            
        }
        

        
};