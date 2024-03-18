/* Copyright 2023 Robert Bosch GmbH and its subsidiaries

 All rights reserved, also regarding any disposal, exploitation, reproduction
 editing, distribution, as well as in the event of applications for industrial
 property rights. */

#include "ccu_bt_tree_nodes/actions/data_service_clients.hpp"

using namespace std::chrono_literals;


namespace ccu_bt_tree_nodes
{
//****************** get ClusterList BT action node *****************************
getClusterList::getClusterList(
    const std::string& name, 
    const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
{
}

BT::NodeStatus getClusterList::tick()
{
    // Waiting for service
    
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("get_cluster_list");
    rclcpp::Client<bautiro_ros_interfaces::srv::GetClusterPosition>::SharedPtr client =
    node->create_client<bautiro_ros_interfaces::srv::GetClusterPosition>("/get_cluster_position");
    auto request = std::make_shared<bautiro_ros_interfaces::srv::GetClusterPosition::Request>();
    request->number_of_next = size;
    while (!client->wait_for_service(1s)) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    auto result = client->async_send_request(request);
             if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS){
        cluster_array = result.get()->cluster_positions;
        array_loaded = true;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Cluster List Loaded");
    }
    else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
        return BT::NodeStatus::FAILURE;
    }
    
    setOutput("poi_pose", cluster_array[idx].pks_pose);
    setOutput("poi_pose_id",cluster_array[idx].cluster_id); 
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending navigation goal with id: %s", cluster_array[idx].cluster_id.c_str());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pose  x: %f, y: %f, z: %f",
                                    cluster_array[idx].pks_pose.position.x,
                                    cluster_array[idx].pks_pose.position.y,
                                    cluster_array[idx].pks_pose.position.z);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Orientation  x: %f, y: %f, z: %f, w: %f",
                                    cluster_array[idx].pks_pose.orientation.x,
                                    cluster_array[idx].pks_pose.orientation.y,
                                    cluster_array[idx].pks_pose.orientation.z,
                                    cluster_array[idx].pks_pose.orientation.w);
    return BT::NodeStatus::SUCCESS;
}   

//****************** get getDrillMask BT action node *****************************
    getDrillMask::getDrillMask(
        const std::string& name, 
        const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
        {
        }

    BT::NodeStatus getDrillMask::tick()
    {
        // Waiting for service
        
        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("get_drill_holes_cluster");
        rclcpp::Client<bautiro_ros_interfaces::srv::GetDrillHolesCluster>::SharedPtr client =
        node->create_client<bautiro_ros_interfaces::srv::GetDrillHolesCluster>("/get_drill_holes_cluster");
        auto request = std::make_shared<bautiro_ros_interfaces::srv::GetDrillHolesCluster::Request>();
        while (!client->wait_for_service(1s)) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }
        auto result = client->async_send_request(request);
       
        if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS){
            setOutput("drill_mask_array", result.get()->drill_masks); 
            return BT::NodeStatus::SUCCESS;
        }  
        else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
            return BT::NodeStatus::FAILURE;
        }
    }   
            

//****************** getDrillMask BT action node *****************************
    getListSize::getListSize(
        const std::string& name, 
        const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
        {
        }

    BT::NodeStatus getListSize::tick()
    {
        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("get_cluster_size");
        rclcpp::Client<bautiro_ros_interfaces::srv::GetClusterPosition>::SharedPtr client =
        node->create_client<bautiro_ros_interfaces::srv::GetClusterPosition>("/get_cluster_position");
        auto request = std::make_shared<bautiro_ros_interfaces::srv::GetClusterPosition::Request>();
     request->number_of_next = 0;
        while (!client->wait_for_service(1s)) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }
        auto result = client->async_send_request(request);
    
        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
        {   
            //size = result.get()->cluster_positions.size();
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Number of cluster %i", (int)result.get()->cluster_positions.size());
            setOutput("list_size", (int)result.get()->cluster_positions.size());
        }
        else 
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
            return BT::NodeStatus::FAILURE;
        
        } 
     return BT::NodeStatus::SUCCESS;
    }   


//****************** addMissionEvent BT action node *****************************
    addMissionEvent::addMissionEvent(
        const std::string& name, 
        const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
        {
        }

    BT::NodeStatus addMissionEvent::tick()
    {
        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_mission_event_client");
        rclcpp::Client<bautiro_ros_interfaces::srv::AddMissionEvent>::SharedPtr client =
        node->create_client<bautiro_ros_interfaces::srv::AddMissionEvent>("/add_mission_event");
        
        auto request = std::make_shared<bautiro_ros_interfaces::srv::AddMissionEvent::Request>();
        getInput("drill_result", request->drill_results);
        getInput("navigation_result", request->navigation_result);
        
        while (!client->wait_for_service(1s)) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Add Mission Event service not available, waiting again...");
        }
        auto result = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result received with response code %ld", result.get()->r_code);
        }
        else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
    }   

//****************** createNavigationResult BT action node *****************************
    createNavigationResult::createNavigationResult(
        const std::string& name, 
        const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
        {
        }

    BT::NodeStatus createNavigationResult::tick()
    {          
        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("create_navigation_result");
        getInput("poi_pose_id",poi_pose_id);
        
        nav_result.cluster_id = poi_pose_id;
        nav_result.nav_goal_reached = true;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "write navigation result to bb  ");
        setOutput("nav_result", nav_result);
        return BT::NodeStatus::SUCCESS;
    }   
            
//****************** createDrillResult BT action node *****************************
    createDrillResult::createDrillResult(
        const std::string& name, 
        const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
        {
        }

    BT::NodeStatus createDrillResult::tick()
    {   
        drill_hole_fb_arr.clear();
        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("create_drill_result");
        getInput("drill_mask_array", drill_mask_arr);
        
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "drill mask size: %i ",(int)drill_mask_arr.size() );
        for (bautiro_ros_interfaces::msg::DrillMask drill_mask: drill_mask_arr)
        {
            for (bautiro_ros_interfaces::msg::DrillHole drill_hole: drill_mask.drill_holes)
            {
            drill_hole_fb.id = drill_hole.id;
            drill_hole_fb.state = 3;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "drill hole id %s", drill_hole_fb.id.c_str());    
            drill_hole_fb_arr.push_back(drill_hole_fb);
            }
        }
        
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "write drill results to bb");
        setOutput("drill_res", drill_hole_fb_arr);
        return BT::NodeStatus::SUCCESS;
    }   

    //****************** addMissionEvent BT action node *****************************
    getMarkerList::getMarkerList(
        const std::string& name, 
        const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
        {
        }

    BT::NodeStatus getMarkerList::tick()
    {
        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("get_marker_list_client");
        rclcpp::Client<bautiro_ros_interfaces::srv::GetMarkersCluster>::SharedPtr client =
        node->create_client<bautiro_ros_interfaces::srv::GetMarkersCluster>("/get_markers_cluster");
        
        auto request = std::make_shared<bautiro_ros_interfaces::srv::GetMarkersCluster::Request>();
        
        while (!client->wait_for_service(1s)) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Marker list service not available, waiting again...");
        }
        auto result = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Number of markers %i", (int)result.get()->markers.size());
            setOutput("marker_array", result.get()->markers);

        }
        else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
    }  

}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ccu_bt_tree_nodes::getDrillMask>("getDrillMask");   
  factory.registerNodeType<ccu_bt_tree_nodes::getMarkerList>("getMarkerList");
  factory.registerNodeType<ccu_bt_tree_nodes::getClusterList>("getPose");
  factory.registerNodeType<ccu_bt_tree_nodes::getListSize>("getSize");
  factory.registerNodeType<ccu_bt_tree_nodes::addMissionEvent>("addMissionEvent");
  factory.registerNodeType<ccu_bt_tree_nodes::createNavigationResult>("createNavResult");
  factory.registerNodeType<ccu_bt_tree_nodes::createDrillResult>("createDrillResult");
}