// Copyright 2024 Robert Bosch GmbH and its subsidiaries
// All rights reserved, also regarding any disposal, exploitation, reproduction
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#include "fpm_celling_distance/ceiling_distance_node.hpp"

#include <memory>
#include <vector>

#include "lifecycle_msgs/msg/transition.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/point_stamped.h"
#include "approximation/linear_regression.hpp"
#include "bautiro_ros_interfaces/srv/get_ceiling_distance.hpp"
#include "ceiling_distance_filter/ceiling_distance_filter.hpp"
#include "ceiling_distance_filter/ceiling_distance_filter_interface.hpp"
#include "point_filters/box_filter.hpp"
#include "point_filters/moving_average_filter.hpp"
#include "point_filters/point2d_filter_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

class CeilingDistanceLifecycleRosNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  CeilingDistanceLifecycleRosNode(
    const std::shared_ptr<ICellingDistanceFilter> distanceFilter,
    bool intra_process_comms = false)
  : rclcpp_lifecycle::LifecycleNode(
      "ceiling_distance",
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)),
    distanceFilter_(distanceFilter) {}

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &)
  {
    service_ =
      this->create_service<bautiro_ros_interfaces::srv::GetCeilingDistance>(
      "get_ceiling_distance",
      std::bind(
        &CeilingDistanceLifecycleRosNode::calculate_distance,
        this, std::placeholders::_1, std::placeholders::_2));

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/rpm/sensors/lift/lidar2d/scan", 10,
      std::bind(
        &CeilingDistanceLifecycleRosNode::lidar_callback, this,
        std::placeholders::_1));

    distance_pub_ =
      this->create_publisher<std_msgs::msg::Float64>("ceiling_distance", 10);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
           CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &)
  {
    sensor_msgs::msg::LaserScan out;
    rclcpp::wait_for_message(
      out, lidar_sub_,
      this->get_node_options().context(),
      std::chrono::seconds(10));
    distance_pub_->on_activate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
           CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &)
  {
    distance_pub_->on_deactivate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
           CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &)
  {
    distance_pub_.reset();
    lidar_sub_.reset();
    service_.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
           CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &)
  {
    distance_pub_.reset();
    lidar_sub_.reset();
    service_.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
           CallbackReturn::SUCCESS;
  }

  void calculate_distance(
    const std::shared_ptr<
      bautiro_ros_interfaces::srv::GetCeilingDistance::Request> request,
    std::shared_ptr<bautiro_ros_interfaces::srv::GetCeilingDistance::Response> response)
  {
    response->distance =
      distanceFilter_->calculate_distance(request->border_points);
  }

  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
  {
    distanceFilter_->update_measurements(*msg);

    geometry_msgs::msg::PointStamped p1;
    geometry_msgs::msg::PointStamped p2;
    p1.point.y = -0.5;
    p2.point.y = 0.5;
    std::vector<geometry_msgs::msg::PointStamped> border_points{p1, p2};
    std_msgs::msg::Float64 measurement;
    measurement.data = distanceFilter_->calculate_distance(border_points);
    distance_pub_->publish(measurement);
  }

public:
  std::shared_ptr<ICellingDistanceFilter> distanceFilter_;

  rclcpp::Service<bautiro_ros_interfaces::srv::GetCeilingDistance>::SharedPtr
    service_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>>
  distance_pub_;
};

// int main(int argc, char * argv[])
// {
//   setvbuf(stdout, NULL, _IONBF, BUFSIZ);
//   // Initialize ROS and create the Node
//   rclcpp::init(argc, argv);
//   auto approximator = std::make_shared<LinearRegression>();
//   auto ceiling_distance_reader =
//   std::make_shared<CeilingDistanceFilter>(approximator);
//   //
//   ceiling_distance_reader->add_points_filter(std::make_shared<BoxFilter>(0.0, 1.0,
//   -2.0, -1.0));
//   //
//   ceiling_distance_reader->add_points_filter(std::make_shared<MovingAverageFilter>(3));

//   auto lc_ceiling_distance_node =
//   std::make_shared<CeilingDistanceLifecycleRosNode>(ceiling_distance_reader);
//   rclcpp::executors::SingleThreadedExecutor exe;
//   exe.add_node(lc_ceiling_distance_node->get_node_base_interface());
//   exe.spin();

//   rclcpp::shutdown();
//   return 0;
// }
