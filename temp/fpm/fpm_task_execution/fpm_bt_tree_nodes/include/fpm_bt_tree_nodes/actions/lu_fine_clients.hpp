// Copyright (c) 2024 Robert Bosch GmbH and its subsidiaries
//
// All rights reserved, also regarding any disposal, exploitation, reproduction,
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#ifndef BEHAVIOR_TREE__PLUGINS__ACTION__MARKER_READ_HPP_
#define BEHAVIOR_TREE__PLUGINS__ACTION__MARKER_READ_HPP_

//Common
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>
#include "behaviortree_cpp_v3/action_node.h"

// ROS includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

// Actions, services, messages
#include <action_msgs/msg/goal_status.hpp>


//Bautiro
#include "bautiro_ros_interfaces/msg/marker.hpp"
#include "bautiro_ros_interfaces/srv/get_markers_cluster.hpp"
#include "lu_fine_localization/srv/lu_fine_localization.hpp"

namespace fpm_bt_tree_nodes {

    /**
    * @brief Action node for reading markers from the blackboard. Inputs:
    *        marker_array (bautiro_ros_interfaces::msg::Marker)
    *
    */
    class MarkerReadBB : public BT::ActionNodeBase 
    {
    public:
        std::vector<bautiro_ros_interfaces::msg::Marker> marker_array;
        MarkerReadBB(const std::string &xml_tag_name,const BT::NodeConfiguration &conf);
        BT::NodeStatus tick() override;


        void halt() override {}

        static BT::PortsList providedPorts() {
            return 
            {
            BT::OutputPort<std::vector<bautiro_ros_interfaces::msg::Marker>>("markers")
            };
    }
    };


    /**
    * @brief Fine Localization. 
    * Inputs:
    *        marker_array (float)
    * Outputs:
    *        pose
    */

    class FineLoc : public BT::AsyncActionNode
    {
        public:
            std::vector<bautiro_ros_interfaces::msg::Marker> marker_array;
            FineLoc(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);
            BT::NodeStatus tick() override;

            void halt() override {} 
            static BT::PortsList providedPorts() {
                return {BT::OutputPort<geometry_msgs::msg::PoseWithCovarianceStamped>("out_pose") , 
                        BT::InputPort<std::vector<bautiro_ros_interfaces::msg::Marker>>("in_markers")
                        };
            }
    };    

} // namespace fpm_bt_tree_nodes

#endif // BEHAVIOR_TREE__PLUGINS__ACTION__MARKER_READ_HPP_