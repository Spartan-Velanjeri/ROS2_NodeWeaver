//  Copyright 2023, 2024 Robert Bosch GmbH and its subsidiaries
//
//  All rights reserved, also regarding any disposal, exploitation, reproduction,
//  editing, distribution, as well as in the event of applications for industrial
//  property rights.


#ifndef RPM_BT_TREE_NODES__ACTIONS__NAV2_POSE_HPP_
#define RPM_BT_TREE_NODES__ACTIONS__NAV2_POSE_HPP_

#include <string>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rpm_bt_tree_nodes/bt_action_node.hpp"


namespace rpm_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::NavigateToPose
 */
class NavigateToPoseAction : public BtActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::NavigateToPoseAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  NavigateToPoseAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose", "Destination to plan to"),
      });
  }
};

}  // namespace rpm_behavior_tree

#endif  // RPM_BT_TREE_NODES__ACTIONS__NAV2_POSE_HPP_
