#ifndef NAV2_BT_NAVIGATOR__BT_NAVIGATOR_HPP_
#define NAV2_BT_NAVIGATOR__BT_NAVIGATOR_HPP_

//common
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <set>
#include <limits>

//ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/odometry_utils.hpp"
#include <nav2_util/geometry_utils.hpp>
#include <nav2_util/robot_utils.hpp>
#include <nav2_behavior_tree/bt_conversions.hpp>

//Bautiro
#include "ccu_behavior_tree/tasks/bt_exec.hpp"





namespace ccu_behavior_tree
{


class CcuBtExecution : public nav2_util::LifecycleNode
{
public:
  
  CcuBtExecution();
 
  ~CcuBtExecution();
protected:
  
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
 

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  // To handle all the BT related execution
  std::unique_ptr<ccu_behavior_tree::CcuExecutor<bautiro_ros_interfaces::action::StartCcuBt>> first_navigator_ ;
  //std::unique_ptr<bt_server::Navigator<bt_tree_nodes::action::SubtractInt>> second_navigator_ ;
  
  ccu_behavior_tree::ExecutionMuxer plugin_muxer_;
  int x_;
  int y_;

};

}  

#endif  // NAV2_BT_NAVIGATOR__BT_NAVIGATOR_HPP_