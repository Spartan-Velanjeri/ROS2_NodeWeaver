#include "hu_bt_tree_nodes/actions/points_handler.hpp"



using namespace std::chrono_literals; 
namespace hu_bt_tree_nodes
{
// struct pattern {
//   struct drillholes[];
//   };
// struct drillholes {
//   geometry_msgs:msg::Pose p;

// };
GetPoi::GetPoi(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  node_ = rclcpp::Node::make_shared("get_poi");
}

BT::NodeStatus GetPoi::tick()
{
  
      std::shared_ptr<rclcpp::Node> node_ = rclcpp::Node::make_shared("load_points");
      std::vector<bautiro_ros_interfaces::msg::DrillMask> array;
      geometry_msgs::msg::Pose p, q;
      int32_t i;

      auto input_handle = getInput<int>("total",total);
      if (!input_handle.has_value()) {
        throw BT::RuntimeError("Missing required input [total]: ",
                           input_handle.error());
      }
      
      input_handle =getInput<int>("point_index", i);
      if (!input_handle.has_value()) {
        throw BT::RuntimeError("Missing required input [total]: ",
                           input_handle.error());
      }
      
      input_handle =getInput<std::vector<bautiro_ros_interfaces::msg::DrillMask>>("point_list", array);
      if (!input_handle.has_value()) {
        throw BT::RuntimeError("Missing required input [point_list]: ",
                           input_handle.error());
      }

      RCLCPP_INFO(node_->get_logger(), "the current index is: %d ", i);
      RCLCPP_INFO(node_->get_logger(), "the total count is: %d ", total);
      RCLCPP_INFO(node_->get_logger(), "Pose Found x: %f y: %f z: %f rx: %f ry: %f rz: %f rw: %f",
        array[i].drill_holes[i].pks_pose.position.x,
        array[i].drill_holes[i].pks_pose.position.y,
        array[i].drill_holes[i].pks_pose.position.z,
        array[i].drill_holes[i].pks_pose.orientation.x,
        array[i].drill_holes[i].pks_pose.orientation.y, 
        array[i].drill_holes[i].pks_pose.orientation.z, 
        array[i].drill_holes[i].pks_pose.orientation.w);

      p.position.x = array[i].drill_holes[i].pks_pose.position.x;
      p.position.y = array[i].drill_holes[i].pks_pose.position.y;
      p.position.z = array[i].drill_holes[i].pks_pose.position.z;
      p.orientation.x = array[i].drill_holes[i].pks_pose.orientation.x;
      p.orientation.y = array[i].drill_holes[i].pks_pose.orientation.y;
      p.orientation.z = array[i].drill_holes[i].pks_pose.orientation.z;
      p.orientation.w = array[i].drill_holes[i].pks_pose.orientation.w;
      
      q.position.x = array[i].drill_holes[i].pks_pose.position.x;
      q.position.y = array[i].drill_holes[i].pks_pose.position.y;
      q.position.z = array[i].drill_holes[i].pks_pose.position.z + 0.07;
      q.orientation.x = array[i].drill_holes[i].pks_pose.orientation.x;
      q.orientation.y = array[i].drill_holes[i].pks_pose.orientation.y;
      q.orientation.z = array[i].drill_holes[i].pks_pose.orientation.z;
      q.orientation.w = array[i].drill_holes[i].pks_pose.orientation.w;
      
      auto output_handle = setOutput("drill_point", p);
      if (!output_handle.has_value()) {
        throw BT::RuntimeError("Missing required output [drill_point]: ",
                           output_handle.error());
      }

      output_handle = setOutput("end_point", q);
      if (!output_handle.has_value()) {
        throw BT::RuntimeError("Missing required output [end_point]: ",
                           output_handle.error());
      }
      
      return BT::NodeStatus::SUCCESS;
}
}  // namespace nav2_behavior_tree
 

namespace hu_bt_tree_nodes
{

UpdatePoi::UpdatePoi(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  node_ = rclcpp::Node::make_shared("update_poi_index");  
}

BT::NodeStatus UpdatePoi::tick()
{
  
      int total;
      int32_t i;
      
      auto input_handle = getInput<int>("total",total);
      if (!input_handle.has_value()) {
        throw BT::RuntimeError("Missing required input [total]: ",
                           input_handle.error());
      }      

      input_handle = getInput<int>("point_index", i);
      if (!input_handle.has_value()) {
        throw BT::RuntimeError("Missing required input [point_index]: ",
                           input_handle.error());
      }
      
      i = i+1;

      auto output_handle = setOutput("update_index", i);
      if (!output_handle.has_value()) {
        throw BT::RuntimeError("Missing required output [update_index]: ",
                           output_handle.error());
      }

      RCLCPP_INFO(node_->get_logger(), "the current index is: %d ", i);
      
      return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree



#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{    
    factory.registerNodeType<hu_bt_tree_nodes::GetPoi>("getpoi");
    factory.registerNodeType<hu_bt_tree_nodes::UpdatePoi>("updatepoi");
}