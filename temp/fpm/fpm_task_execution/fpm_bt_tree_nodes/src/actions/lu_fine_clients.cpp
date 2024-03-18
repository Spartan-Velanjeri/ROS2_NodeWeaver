// Copyright (c) 2024 Robert Bosch GmbH and its subsidiaries
//
// All rights reserved, also regarding any disposal, exploitation, reproduction,
// editing, distribution, as well as in the event of applications for industrial
// property rights.


#include "fpm_bt_tree_nodes/actions/lu_fine_clients.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

 
using namespace std::chrono_literals; 


namespace fpm_bt_tree_nodes
{

    //=======================================================
    // MarkerReadBB =========================================
    //=======================================================

    MarkerReadBB::MarkerReadBB(const std::string &name,const BT::NodeConfiguration &conf):BT::ActionNodeBase(name, conf){}

    BT::NodeStatus MarkerReadBB::tick()
    {
        rclcpp::Node::SharedPtr node_ = rclcpp::Node::make_shared("marker_read_bb");
        node_->declare_parameter("client_wait_time", 2);

        // Create action client for the lift driver service
        rclcpp::Client<bautiro_ros_interfaces::srv::GetMarkersCluster>::SharedPtr client 
            = node_->create_client<bautiro_ros_interfaces::srv::GetMarkersCluster>("/get_markers_cluster");
            
        if (!client->wait_for_service(std::chrono::seconds(node_->get_parameter("client_wait_time").as_int())))
        {
            RCLCPP_ERROR(node_->get_logger(), "Marker service not available.");
            return BT::NodeStatus::FAILURE;
        }

        // read markers from blackboard 
        // auto reading_markers = getInput("markers", marker_array);

        // if (marker_array.size() > 0 ) {
        //     throw BT::RuntimeError(
        //         "Missing required input [markers]: ",
        //         reading_markers.error()
        //         );
        // }

        // Prepare request for service
        auto request = std::make_shared<bautiro_ros_interfaces::srv::GetMarkersCluster::Request>();

        // Call service
        auto result = client->async_send_request(request);

        // wait until result is available
        if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS){ // Service call successful
            RCLCPP_INFO(node_->get_logger(), "service call successful.");
            setOutput("out_markers", result.get()->markers);
            return BT::NodeStatus::SUCCESS;
        }
        else { //Service call failed
            RCLCPP_ERROR(node_->get_logger(), "service call failed.");
            return BT::NodeStatus::FAILURE;
        }
    }


    //=======================================================
    // Fine localization ====================================
    //=======================================================

    FineLoc::FineLoc(const std::string & name, const BT::NodeConfiguration & conf):BT::AsyncActionNode(name, conf){}

    inline BT::NodeStatus FineLoc::tick()
    {
        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("fineloc_client");
        rclcpp::Client<lu_fine_localization::srv::LuFineLocalization>::SharedPtr client 
                = node->create_client<lu_fine_localization::srv::LuFineLocalization>("/fineloc");
        
        while (!client->wait_for_service(1s)) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "FineLoc service not available, waiting again...");
        }

        // getInput("in_markers",in_arr);
        // read markers from blackboard 
        auto reading_markers = getInput("markers", marker_array);

        if (marker_array.size() < 3 ) {
            throw BT::RuntimeError(
                "Missing required input [markers]: ",
                reading_markers.error()
                );
        }        

        auto request = std::make_shared<lu_fine_localization::srv::LuFineLocalization::Request>();
        request->markers = marker_array;

        auto result = client->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
            setOutput("out_pose", result.get()->pose);
            return BT::NodeStatus::SUCCESS;
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
            return BT::NodeStatus::FAILURE;
        }
    }    

}  // namespace fpm_bt_tree_nodes


BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<fpm_bt_tree_nodes::MarkerReadBB>("MarkerReadBB");
  factory.registerNodeType<fpm_bt_tree_nodes::FineLoc>("FineLoc");
}