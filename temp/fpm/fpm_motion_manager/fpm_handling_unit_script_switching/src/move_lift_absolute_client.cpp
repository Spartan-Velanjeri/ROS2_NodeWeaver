#include <functional>
#include <memory>
#include <thread>
//Actions, services, messages
#include "bautiro_ros_interfaces/action/move_lift_absolute.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

//ROS includes
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/parameter.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

class MoveLiftAbsoluteClient: public BT::AsyncActionNode
  {
  public:
    
    
    MoveLiftAbsoluteClient(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
    
    }
    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<float32>("requested_target_lift_level")};
    }

    BT::NodeStatus tick() override
    {
      rclcpp::Node::SharedPtr node_;
      node_ = rclcpp::Node::make_shared("start_action");
      auto action_client = rclcpp_action::create_client<bautiro_ros_interfaces::action::MoveLiftAbsolute>(node_, "/start");
      if (!action_client->wait_for_action_server(std::chrono::seconds(20))) {
            return BT::NodeStatus::FAILURE;
        }
      auto goal_msg = bautiro_ros_interfaces::action::MoveLiftAbsolute::Goal();
      getInput<int32_t>("requested_target_lift_level",goal_msg.requested_target_lift_level);
      auto goal_handle_future = action_client->async_send_goal(goal_msg);
      if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
                rclcpp::FutureReturnCode::SUCCESS)
        {
            
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "send goal call failed");
            return BT::NodeStatus::FAILURE;
        }
        rclcpp_action::ClientGoalHandle<bautiro_ros_interfaces::action::MoveLiftAbsolute>::SharedPtr goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal was rejected by server");
            return BT::NodeStatus::FAILURE;
        }
        
        
        auto result_future = action_client->async_get_result(goal_handle);
        
        if (rclcpp::spin_until_future_complete(node_, result_future) !=
                rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "get result call failed " );
            return BT::NodeStatus::FAILURE;
        }
        rclcpp_action::ClientGoalHandle<bautiro_ros_interfaces::action::MoveLiftAbsolute>::WrappedResult wrapped_result = result_future.get();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "result received");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "the response code is %d" ,wrapped_result.result->response_code);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "the final lift level is %f" ,wrapped_result.result->final_lift_level);

        
        return BT::NodeStatus::SUCCESS;
    }
  };