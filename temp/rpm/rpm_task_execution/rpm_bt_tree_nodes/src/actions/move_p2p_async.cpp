//  Copyright 2024 Robert Bosch GmbH and its subsidiaries
//
//  All rights reserved, also regarding any disposal, exploitation, reproduction,
//  editing, distribution, as well as in the event of applications for industrial
//  property rights.

#include "rpm_bt_tree_nodes/actions/move_p2p_async.hpp"


namespace rpm_behavior_tree
{
  int status_call;
  int status_msg;
  std::mutex status_mutex;
  std::condition_variable status_cv;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr subscription_;

  ActionSubscriber::ActionSubscriber():Node("actionsubscriber")
  {
    subscription_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
      "/move_ur_base/_action/status",  // Change this to the topic you want to subscribe to
      10,         // Queue size
      std::bind(&ActionSubscriber::status_callback, this, std::placeholders::_1)
      );
  }

  void ActionSubscriber::status_callback(action_msgs::msg::GoalStatusArray::SharedPtr msg)
  {
    auto status_list = msg->status_list;
    status_call = status_list.back().status;
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Received Action Status: %d", status_msg);
  }
  BT::NodeStatus Move_p2p_async::onStart()
  {
    rclcpp::Node::SharedPtr node_p2p = rclcpp::Node::make_shared("navigate_to_pose_p2p");
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Start MoveP2p Tick!!!");
    auto action_client = rclcpp_action::create_client<
      bautiro_ros_interfaces::action::MoveUrBase>(node_p2p, "/move_ur_base");
    if (!action_client->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "P2P action send goal call failed");
        return BT::NodeStatus::FAILURE;
    }
    auto goal_msg = bautiro_ros_interfaces::action::MoveUrBase::Goal();
    geometry_msgs::msg::PoseStamped p;
    getInput<geometry_msgs::msg::PoseStamped>("goal_pose", p);
    goal_msg.target_pose = p.pose;
    std::thread subscriber_thread([node_p2p]() {
        rclcpp::spin(std::make_shared<ActionSubscriber>());
    });
    subscriber_thread.detach();
    status_msg = status_call;
    auto goal_handle_future = action_client->async_send_goal(goal_msg);
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "P2P_Action_Status: %d", status_msg);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus Move_p2p_async::onRunning()
  {
    status_msg = status_call;
     RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "P2P_Action_Status: %d", status_msg);
     if (status_msg <= 2)
     {
         return BT::NodeStatus::RUNNING;
     } else if (status_msg == 4) {
         RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "P2P_Action_Status: %d", status_msg);
         return BT::NodeStatus::SUCCESS;
     } else {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "P2P_Action_Status: %d", status_msg);
        return BT::NodeStatus::FAILURE;
     }
  }

  void Move_p2p_async::onHalted()
    {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Move_p2p ABORTED");
    }

  }  // namespace rpm_behavior_tree

#include <string>
#include <memory>
#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<rpm_behavior_tree::Move_p2p_async>(
        name, config);
    };

  factory.registerBuilder<rpm_behavior_tree::Move_p2p_async>(
    "Movep2p", builder);
}
