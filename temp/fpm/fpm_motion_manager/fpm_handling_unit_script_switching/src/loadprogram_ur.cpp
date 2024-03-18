#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <ur_dashboard_msgs/srv/load.hpp>
#include <unistd.h>
#include <chrono>
#include <thread>
using namespace std::chrono_literals;
class LoadProgram : public BT::AsyncActionNode
  {
  public:
      bool x;
      rclcpp::Node::SharedPtr node_;
      explicit LoadProgram(const std::string& name, const BT::NodeConfiguration& config)
          : BT::AsyncActionNode(name, config)
      {
      }

      static BT::PortsList providedPorts() { return {BT::InputPort<std::string>("file_name")}; }
      
      
      BT::NodeStatus tick() override {
        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("loading_the_program");
        rclcpp::Client<ur_dashboard_msgs::srv::Load>::SharedPtr client =
        node->create_client<ur_dashboard_msgs::srv::Load>("/dashboard_client/load_program");
        auto request = std::make_shared<ur_dashboard_msgs::srv::Load::Request>();
        //request->filename = "demo.urp";
        getInput<std::string>("file_name",request->filename);
        while (!client->wait_for_service(1s)) {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }
        auto result = client->async_send_request(request);
        //auto results = std::make_shared<ur_dashboard_msgs::srv::GetRobotMode::Response>();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Done");
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "the value is %1d", results->robot_mode.mode);
        if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
        {
          
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", result.get()->answer.c_str());
            return BT::NodeStatus::SUCCESS;
          }  
        else {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
        }
      }   
      
};

class Loadinstallation : public BT::AsyncActionNode
  {
  public:
      bool x;
      rclcpp::Node::SharedPtr node_;
      explicit Loadinstallation(const std::string& name, const BT::NodeConfiguration& config)
          : BT::AsyncActionNode(name, config)
      {
      }

      static BT::PortsList providedPorts() { return {}; }
      
      
      BT::NodeStatus tick() override {
        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("loading_the_installation");
        rclcpp::Client<ur_dashboard_msgs::srv::Load>::SharedPtr client =
        node->create_client<ur_dashboard_msgs::srv::Load>("/dashboard_client/load_installation");
        auto request = std::make_shared<ur_dashboard_msgs::srv::Load::Request>();
        request->filename = 'demo.urp';
        while (!client->wait_for_service(1s)) {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }
        auto result = client->async_send_request(request);
        //auto results = std::make_shared<ur_dashboard_msgs::srv::GetRobotMode::Response>();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Done");
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "the value is %1d", results->robot_mode.mode);
        if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
        {
          
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", result.get()->answer.c_str());
            return BT::NodeStatus::SUCCESS;
          }   
        else {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
        }
      }  
      
  };