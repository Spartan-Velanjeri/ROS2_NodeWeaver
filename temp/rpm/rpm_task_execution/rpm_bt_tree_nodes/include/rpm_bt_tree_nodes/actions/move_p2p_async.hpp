//  Copyright 2024 Robert Bosch GmbH and its subsidiaries
//
//  All rights reserved, also regarding any disposal, exploitation, reproduction,
//  editing, distribution, as well as in the event of applications for industrial
//  property rights.


#ifndef RPM_BT_TREE_NODES__ACTIONS__MOVE_P2P_ASYNC_HPP_
#define RPM_BT_TREE_NODES__ACTIONS__MOVE_P2P_ASYNC_HPP_


#include <string>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rpm_bt_tree_nodes/bt_action_node.hpp"
#include "bautiro_ros_interfaces/action/move_base_link.hpp"
#include "bautiro_ros_interfaces/action/move_ur_base.hpp"

namespace rpm_behavior_tree
{

/**
 * @brief A rpm_behavior_tree::BtActionNode class that wraps bautiro_ros_interfaces::action::MoveUrBase
 */
class Move_p2p_async : public BT::StatefulActionNode
    {
     public:
        Move_p2p_async(const std::string& name, const BT::NodeConfiguration& config)
            : StatefulActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
          return
            {
            BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose")
            };
        }

        BT::NodeStatus onStart() override;

        BT::NodeStatus onRunning() override;

        void onHalted() override;
    };

class ActionSubscriber: public rclcpp::Node
    {
        public:
            ActionSubscriber();
            void status_callback(action_msgs::msg::GoalStatusArray::SharedPtr msg);
    };

}  // namespace rpm_behavior_tree

#endif  // RPM_BT_TREE_NODES__ACTIONS__MOVE_P2P_ASYNC_HPP_
