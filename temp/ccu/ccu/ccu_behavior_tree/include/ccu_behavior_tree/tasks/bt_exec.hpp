#ifndef NAV2_BT_NAVIGATOR__BT_EXEC_HPP_
#define NAV2_BT_NAVIGATOR__BT_EXEC_HPP_

//common
#include <string>
#include <vector>
#include <memory>
#include <set>
#include <limits>
#include <ament_index_cpp/get_package_share_directory.hpp>

//ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <std_msgs/msg/int32.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/odometry_utils.hpp"
//#include "nav2_bt_navigator/navigator.hpp"

//Bautiro
#include "bautiro_ros_interfaces/action/start_ccu_bt.hpp"
#include "ccu_behavior_tree/ccu_executor.hpp"


namespace ccu_behavior_tree
{


class CcuCluster
  : public ccu_behavior_tree::CcuExecutor<bautiro_ros_interfaces::action::StartCcuBt>
{
public:
  using ActionT = bautiro_ros_interfaces::action::StartCcuBt;

  
  CcuCluster()
  : CcuExecutor() {}

  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

  
  bool cleanup() override;

  

  std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;
  // std::string getDefaultBTFilename() override;
  std::string getName() {return std::string("ccu_start");}

  

protected:
  
  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;

 
  void onLoop() override;
  void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;
  
  void goalCompleted(typename ActionT::Result::SharedPtr result) override;

  

  rclcpp::Time start_time_;

  rclcpp_action::Client<ActionT>::SharedPtr self_client_;

  std::string goal_blackboard_id_;

};

}  

#endif  