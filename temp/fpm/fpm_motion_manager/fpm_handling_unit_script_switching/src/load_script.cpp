#include <rclcpp/rclcpp.hpp>
#include <ur_msgs/srv/set_auxiliary_script_arguments.hpp>
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <unistd.h>
#include <chrono>
#include <thread>
using namespace std::chrono_literals; 
class LoadScript : public BT::AsyncActionNode
{
public:
    bool x;
    //rclcpp::Node::SharedPtr node_;
    explicit LoadScript(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts() { return {}; }
    
    
    BT::NodeStatus tick() override
    {
      
      
      std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("load_script");
      rclcpp::Client<ur_msgs::srv::SetAuxiliaryScriptArguments>::SharedPtr client =
      node->create_client<ur_msgs::srv::SetAuxiliaryScriptArguments>("/io_and_status_controller/set_aux_script_arguments");
      auto request = std::make_shared<ur_msgs::srv::SetAuxiliaryScriptArguments::Request>();
      request->drill_pos_x_replace = -0.2;
      request->drill_pos_y_replace = -0.55;
      request->drill_pos_z_replace = -0.20;
      request->drill_depth_replace = 0.1;
      while (!client->wait_for_service(1s)) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }
      //int emptyarray[4];
      //request->all_arguments = emptyarray;
      auto result = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Script loaded");
          return BT::NodeStatus::SUCCESS;
        } else {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "failed to load script");
          return BT::NodeStatus::FAILURE;
        }
    
    }
};