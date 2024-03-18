
#include "hu_bt_tree_nodes/actions/motion_manager_clients.hpp"

namespace hu_bt_tree_nodes
{

ConfiguredPoseClient::ConfiguredPoseClient(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  node_ = rclcpp::Node::make_shared(node_name_);
  //Parameter
  node_->declare_parameter("client_wait_time", 20);
  //Clients
  action_client_ = rclcpp_action::create_client<bautiro_ros_interfaces::action::SetHandlingUnitConfiguredPose>(node_, server_name_);

}

BT::NodeStatus ConfiguredPoseClient::tick()
{

  //Wait until server is available
  if (!action_client_->wait_for_action_server(std::chrono::seconds(
      node_->get_parameter("client_wait_time").as_int()))){
    return BT::NodeStatus::FAILURE;
  }
  //read  and check input
  auto input_handle = getInput<int32_t>("handling_unit_configured_pose",goal_msg_.handling_unit_configured_pose);
  if (!input_handle.has_value()) {
    throw BT::RuntimeError("Missing required input [x]: ",input_handle.error());}
  //send async goal to server
  auto goal_handle_future = action_client_->async_send_goal(goal_msg_);
  
  RCLCPP_INFO(node_->get_logger(), "sending goal id %d", goal_msg_.handling_unit_configured_pose);

  if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS){    
    RCLCPP_INFO(node_->get_logger(), "send goal call failed");
    return BT::NodeStatus::FAILURE;
  }

  goal_handle_ = goal_handle_future.get();
  if (!goal_handle_) {    
    RCLCPP_INFO(node_->get_logger(), "Goal was rejected by server");
    return BT::NodeStatus::FAILURE;
  }
    
    
  auto result_future = action_client_->async_get_result(goal_handle_);
    
  if (rclcpp::spin_until_future_complete(node_, result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
      RCLCPP_ERROR(node_->get_logger(), "get result call failed " );
      return BT::NodeStatus::FAILURE;
  }
  
  wrapped_result_ = result_future.get();
  RCLCPP_INFO(node_->get_logger(), "result received");
  RCLCPP_INFO(node_->get_logger(), "the response code is %d" ,wrapped_result_.result->response_code);
    
  return BT::NodeStatus::SUCCESS;
}

}  // namespace hu_bt_tree_nodes



namespace hu_bt_tree_nodes
{
MoveAbsoluteClient::MoveAbsoluteClient(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  node_ = rclcpp::Node::make_shared(node_name_);
  //Parameter
  node_->declare_parameter("client_wait_time", 20);
  //Server, Clients
  action_client_ = rclcpp_action::create_client<bautiro_ros_interfaces::action::MoveHandlingUnitAbsolute>(node_, server_name_);

}



inline BT::NodeStatus MoveAbsoluteClient::tick()
{
  //Wait until server is available
  if (!action_client_->wait_for_action_server(std::chrono::seconds(
      node_->get_parameter("client_wait_time").as_int()))){
    return BT::NodeStatus::FAILURE;
  }

  //Read & check input: absolute_target_position
  auto input_handle = getInput<geometry_msgs::msg::PoseStamped>("absolute_target_position",goal_msg_.absolute_target_position);
  if (!input_handle.has_value()) {
    throw BT::RuntimeError("Missing required input [absolute_target_position]: ",
                           input_handle.error());
  }

  //send action goal
  auto goal_handle_future = action_client_->async_send_goal(goal_msg_);
  //wait until action is finished
  if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node_->get_logger(), "send goal call failed");
        return BT::NodeStatus::FAILURE;
    }
    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
        
        RCLCPP_INFO(node_->get_logger(), "Goal was rejected by server");
        return BT::NodeStatus::FAILURE;
    }
    
    
  auto result_future = action_client_->async_get_result(goal_handle);
  
  if (rclcpp::spin_until_future_complete(node_, result_future) !=
          rclcpp::FutureReturnCode::SUCCESS)
  {
      RCLCPP_ERROR(node_->get_logger(), "get result call failed " );
      return BT::NodeStatus::FAILURE;
  }
  
  wrapped_result_ = result_future.get();
  RCLCPP_INFO(node_->get_logger(), "result received");
  RCLCPP_INFO(node_->get_logger(), "the response code is %d" ,wrapped_result_.result->response_code);
     
  return BT::NodeStatus::SUCCESS;
}

}  // namespace hu_bt_tree_nodes


namespace hu_bt_tree_nodes
{
MoveRelativeClient::MoveRelativeClient(
  const std::string & name,
  const BT::NodeConfiguration & conf)
:  BT::ActionNodeBase(name, conf)
{
  node_ = rclcpp::Node::make_shared(node_name_);
  //Parameter
  node_->declare_parameter("client_wait_time", 20);
  //Server, Clients
  action_client_ = rclcpp_action::create_client<bautiro_ros_interfaces::action::MoveHandlingUnitRelativToWorkplane>(node_, server_name_);

}

inline BT::NodeStatus MoveRelativeClient::tick()
{

    if (!action_client_->wait_for_action_server(std::chrono::seconds(20))) {
          return BT::NodeStatus::FAILURE;
      }
    //Read inputs and handle input errors
    auto input_handle = getInput<float>("x",x);
    if (!input_handle.has_value()) {
      throw BT::RuntimeError("Missing required input [x]: ",
                             input_handle.error());
    }
    input_handle = getInput<float>("y", y);
    if (!input_handle.has_value()) {
      throw BT::RuntimeError("Missing required input [y]: ",
                             input_handle.error());
    }

    goal_msg_.relative_target_position={x,y};

    //Send async goals
    auto goal_handle_future = action_client_->async_send_goal(goal_msg_);

    if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
              rclcpp::FutureReturnCode::SUCCESS)
    {
        
        RCLCPP_INFO(node_->get_logger(), "send goal call failed");
        return BT::NodeStatus::FAILURE;
    }

    goal_handle_ = goal_handle_future.get();
    
    if (!goal_handle_) {
        
        RCLCPP_INFO(node_->get_logger(), "Goal was rejected by server");
        return BT::NodeStatus::FAILURE;
    }
    
    auto result_future = action_client_->async_get_result(goal_handle_);
    
    if (rclcpp::spin_until_future_complete(node_, result_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "get result call failed " );
        return BT::NodeStatus::FAILURE;
    }
    wrapped_result_ = result_future.get();
    RCLCPP_INFO(node_->get_logger(), "result received");
    RCLCPP_INFO(node_->get_logger(), "the response code is %d" ,wrapped_result_.result->response_code);
        
    return BT::NodeStatus::SUCCESS;
}

}  // namespace hu_bt_tree_nodes

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<hu_bt_tree_nodes::ConfiguredPoseClient>("ConfiguredPose");
    factory.registerNodeType<hu_bt_tree_nodes::MoveAbsoluteClient>("MoveAbsolute");
    factory.registerNodeType<hu_bt_tree_nodes::MoveRelativeClient>("MoveRelative");
}