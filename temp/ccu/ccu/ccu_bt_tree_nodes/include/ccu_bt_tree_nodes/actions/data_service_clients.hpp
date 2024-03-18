/* Copyright 2023 Robert Bosch GmbH and its subsidiaries

 All rights reserved, also regarding any disposal, exploitation, reproduction
 editing, distribution, as well as in the event of applications for industrial
 property rights. */



#ifndef BEHAVIOR_TREE__PLUGINS__ACTION__DATA_SERVICE_NODES
#define BEHAVIOR_TREE__PLUGINS__ACTION__DATA_SERVICE_NODES


#include <unistd.h>

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>

//ROS
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/bool.hpp"

//BAUTIRO
#include "bautiro_ros_interfaces/srv/get_cluster_position.hpp"
#include "bautiro_ros_interfaces/srv/get_cluster_pose.hpp"
#include "bautiro_ros_interfaces/srv/get_drill_holes_cluster.hpp"
#include "bautiro_ros_interfaces/srv/get_markers_cluster.hpp"
#include "bautiro_ros_interfaces/srv/add_mission_event.hpp"
#include "bautiro_ros_interfaces/msg/marker.hpp"
#include "bautiro_ros_interfaces/msg/drill_mask.hpp"
#include "bautiro_ros_interfaces/msg/drill_hole.hpp"
#include "bautiro_ros_interfaces/msg/navigation_result.hpp"
#include "bautiro_ros_interfaces/msg/cluster_position.hpp"

namespace ccu_bt_tree_nodes
{
    //****************** get cluster list BT action node *****************************
    class getClusterList: public BT::SyncActionNode
    {
        public:
            // bool start = false; 
            bool drill_job_complete = true; 
            bool motion_cluster_done = false; 
            int idx = 0;
            int size = 1;
            bool array_loaded = false;
            std::vector<bautiro_ros_interfaces::msg::ClusterPosition> cluster_array;

            getClusterList(
                const std::string& name,
                const BT::NodeConfiguration& config);

            BT::NodeStatus tick() override;

            static BT::PortsList providedPorts()
            {
                return{
                    BT::OutputPort<geometry_msgs::msg::Pose>("poi_pose"),
                    BT::OutputPort<std::string>("poi_pose_id")
                    };
            }

    
    };

//****************** getDrillMask BT action node *****************************
    class getDrillMask : public BT::SyncActionNode
    {
        public:
            getDrillMask(
                const std::string& name,
                const BT::NodeConfiguration& config);

            BT::NodeStatus tick() override;

            static BT::PortsList providedPorts()
            {
                return{
                    BT::OutputPort<std::vector<bautiro_ros_interfaces::msg::DrillMask>>("drill_mask_array")
                    };
            }

    };
//****************** getListSize BT action node *****************************
    class getListSize : public BT::SyncActionNode
    {
        public:

            getListSize(
                const std::string& name,
                const BT::NodeConfiguration& config);

            BT::NodeStatus tick() override;

            static BT::PortsList providedPorts()
            {
                return{
                        BT::OutputPort<int>("list_size")
                    };
            }
    };
//****************** addMissionEvent BT action node *****************************
    class addMissionEvent : public BT::SyncActionNode
    {
        public:

            addMissionEvent(
                const std::string& name,
                const BT::NodeConfiguration& config);

            BT::NodeStatus tick() override;

            static BT::PortsList providedPorts()
            {
                return{
                    BT::InputPort<std::vector<bautiro_ros_interfaces::msg::DrillHoleFeedback>>("drill_result"),
                    BT::InputPort<bautiro_ros_interfaces::msg::NavigationResult>("navigation_result")
                    };
            }
    };


    class createNavigationResult : public BT::SyncActionNode
    {
    public:
        std::string poi_pose_id;
        bautiro_ros_interfaces::msg::NavigationResult nav_result;

        createNavigationResult(
            const std::string& name,
            const BT::NodeConfiguration& config);
        
        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts()
        {
            return{
                BT::InputPort<std::string>("poi_pose_id"),
                BT::OutputPort<bautiro_ros_interfaces::msg::NavigationResult>("nav_result")                
                };
        }
    
    };

    class createDrillResult : public BT::SyncActionNode
    {
    public:

        std::vector<bautiro_ros_interfaces::msg::DrillMask>  drill_mask_arr;
        std::vector<bautiro_ros_interfaces::msg::DrillHoleFeedback>  drill_hole_fb_arr;
        bautiro_ros_interfaces::msg::DrillHoleFeedback drill_hole_fb;

        createDrillResult(
        const std::string& name,
        const BT::NodeConfiguration& config);

        BT::NodeStatus tick() override;
        static BT::PortsList providedPorts()
        {
            return{
                BT::InputPort<std::vector<bautiro_ros_interfaces::msg::DrillMask>>("drill_mask_array"),
                BT::OutputPort<std::vector<bautiro_ros_interfaces::msg::DrillHoleFeedback>>("drill_res")
                };
        }
    };

    class getMarkerList : public BT::SyncActionNode
    {
    public:

        std::vector<bautiro_ros_interfaces::msg::Marker>  marker_arr;

        getMarkerList(
        const std::string& name,
        const BT::NodeConfiguration& config);

        BT::NodeStatus tick() override;
        static BT::PortsList providedPorts()
        {
            return{
                BT::OutputPort<std::vector<bautiro_ros_interfaces::msg::Marker>>("marker_array")
                };
        }
    };

} // end namespace ccu_bt_tree_nodes

#endif  // BEHAVIOR_TREE__PLUGINS__ACTION__START_MISSION