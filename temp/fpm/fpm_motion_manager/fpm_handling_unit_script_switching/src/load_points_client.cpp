#include <rclcpp/rclcpp.hpp>
#include <bautiro_ros_interfaces/srv/get_drill_holes_cluster.hpp>
#include <bautiro_ros_interfaces/msg/drill_mask.hpp>
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <unistd.h>

class LoadPoints : public BT::SyncActionNode
{
    public:
        LoadPoints(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return{ BT::OutputPort<std::vector<bautiro_ros_interfaces::msg::DrillMask>>("pointslist") , BT::OutputPort<int32_t>("totalpoints")};
        }

        virtual BT::NodeStatus tick() override
        {
            // Waiting for service
            std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("points_load");
            rclcpp::Client<bautiro_ros_interfaces::srv::GetDrillHolesCluster>::SharedPtr client =
            node->create_client<bautiro_ros_interfaces::srv::GetDrillHolesCluster>("/get_poi_list");
            auto request = std::make_shared<bautiro_ros_interfaces::srv::GetDrillHolesCluster::Request>();
            while (!client->wait_for_service(1s)) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
            }
            auto result = client->async_send_request(request);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Done");
            if (rclcpp::spin_until_future_complete(node, result) ==
            rclcpp::FutureReturnCode::SUCCESS)
            {
                setOutput("pointslist", result.get()->drill_masks);
                setOutput("totalpoints", result.get()->drill_masks.size());
                // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", result.get()->success_msg.c_str());
                return BT::NodeStatus::SUCCESS;
            }  
            else {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
                return BT::NodeStatus::FAILURE;

            }
        }   
            
};