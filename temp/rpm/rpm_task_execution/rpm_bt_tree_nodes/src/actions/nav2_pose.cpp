// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>

#include "rpm_bt_tree_nodes/actions/nav2_pose.hpp"

namespace rpm_behavior_tree
{

NavigateToPoseAction::NavigateToPoseAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name, conf)
{
}

void NavigateToPoseAction::on_tick()
{
  if (!getInput("target_pose", goal_.pose)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "NavigateToPoseAction: goal not provided");
  } else {
    getInput<geometry_msgs::msg::PoseStamped>("target_pose", goal_.pose);
    goal_.pose.header.frame_id = "map";
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "In nav2_pose");
    return;
  }
}

}  // namespace rpm_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<rpm_behavior_tree::NavigateToPoseAction>(
        name, "navigate_to_pose", config);
    };

  factory.registerBuilder<rpm_behavior_tree::NavigateToPoseAction>(
    "NavigateToPose", builder);
}
