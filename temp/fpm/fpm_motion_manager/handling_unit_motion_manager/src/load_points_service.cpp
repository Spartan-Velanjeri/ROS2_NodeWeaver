#include "rclcpp/rclcpp.hpp"
#include <yaml-cpp/yaml.h>
#include <bautiro_ros_interfaces/srv/get_drill_holes_cluster.hpp>
#include <bautiro_ros_interfaces/msg/drill_mask.hpp>
#include <bautiro_ros_interfaces/msg/drill_hole.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <atomic>
#include <memory>
#include <vector>

// std::atomic<unsigned int> poi_index{0};
double points[100][3];

int32_t i = 0;

std::string map_filename = ament_index_cpp::get_package_share_directory("fpm_handling_unit_script_switching") + "/config/cluster_points.yaml";
void add(const std::shared_ptr<bautiro_ros_interfaces::srv::GetDrillHolesCluster::Request> request,
          std::shared_ptr<bautiro_ros_interfaces::srv::GetDrillHolesCluster::Response>      response)
{

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request");
  YAML::Node map_config_file = YAML::LoadFile(map_filename.c_str());

  YAML::Node loaded_yaml_poses = std::move(map_config_file["pois"]);
  std::vector<bautiro_ros_interfaces::msg::DrillMask> drill_mask_arr;
  bautiro_ros_interfaces::msg::DrillMask drill_mask;
  std::vector<bautiro_ros_interfaces::msg::DrillHole> drill_holes;
  bautiro_ros_interfaces::msg::DrillHole drill_hole;
  for (YAML::const_iterator it = loaded_yaml_poses.begin(); it != loaded_yaml_poses.end(); ++it) {
    //auto poi_label = it->second["label"].as<std::string>();
    auto loaded_pose = it->second["pose"];

    drill_hole.pks_pose.position.x = loaded_pose[0][0].as<double>();
    drill_hole.pks_pose.position.y = loaded_pose[0][1].as<double>();
    drill_hole.pks_pose.position.z = loaded_pose[0][2].as<double>();
    drill_hole.pks_pose.orientation.x = loaded_pose[1][0].as<double>();
    drill_hole.pks_pose.orientation.y = loaded_pose[1][1].as<double>();
    drill_hole.pks_pose.orientation.z = loaded_pose[1][2].as<double>();
    drill_hole.pks_pose.orientation.w = loaded_pose[1][3].as<double>();

    drill_holes.push_back(drill_hole);
    i = i + 1;
  }
  drill_mask.drill_holes = drill_holes;
  drill_mask_arr.push_back(drill_mask);
  response->drill_masks = drill_mask_arr;  
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("poi_list_server");

  rclcpp::Service<bautiro_ros_interfaces::srv::GetDrillHolesCluster>::SharedPtr service =
    node->create_service<bautiro_ros_interfaces::srv::GetDrillHolesCluster>("get_drill_holes_cluster", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to send Drill Mask.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}