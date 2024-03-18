// Copyright 2024 Robert Bosch GmbH and its subsidiaries
// All rights reserved, also regarding any disposal, exploitation, reproduction
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#include "fpm_celling_distance/ceiling_distance_node.hpp"

#include <memory>
#include <vector>

#include "bautiro_ros_interfaces/srv/get_ceiling_distance.hpp"
#include "approximation/linear_regression.hpp"
#include "ceiling_distance_filter/ceiling_distance_filter.hpp"
#include "ceiling_distance_filter/ceiling_distance_filter_interface.hpp"
#include "point_filters/box_filter.hpp"
#include "point_filters/moving_average_filter.hpp"
#include "point_filters/point2d_filter_interface.hpp"
#include "geometry_msgs/msg/point_stamped.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64.hpp"

class CeilingDistanceRosNode : public rclcpp::Node
{
public:
  CeilingDistanceRosNode(
    const std::shared_ptr<ICellingDistanceFilter> distanceFilter)
  : Node("ceiling_distance"), distanceFilter_(distanceFilter)
  {
    // onConfigure
    this->declare_parameter("border_points", std::vector<double>({1, 0}));

    service_ =
      this->create_service<bautiro_ros_interfaces::srv::GetCeilingDistance>(
      "get_ceiling_distance",
      std::bind(
        &CeilingDistanceRosNode::calculate_distance, this,
        std::placeholders::_1, std::placeholders::_2));

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/rpm/sensors/lift/lidar2d/scan", 10,
      std::bind(
        &CeilingDistanceRosNode::lidar_callback, this,
        std::placeholders::_1));
    distance_pub_ =
      this->create_publisher<std_msgs::msg::Float64>("ceiling_distance", 10);
  }

  void calculate_distance(
    const std::shared_ptr<
      bautiro_ros_interfaces::srv::GetCeilingDistance::Request>
    request,
    std::shared_ptr<bautiro_ros_interfaces::srv::GetCeilingDistance::Response>
    response)
  {
    response->distance =
      distanceFilter_->calculate_distance(request->border_points);
  }

  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
  {
    distanceFilter_->update_measurements(*msg);

    auto points_y = this->get_parameter("border_points").as_double_array();
    std::vector<geometry_msgs::msg::PointStamped> border_points;

    for (size_t i = 0; i < points_y.size(); ++i) {
      geometry_msgs::msg::PointStamped p;
      p.point.y = points_y[i];
      border_points.push_back(p);
    }

    std_msgs::msg::Float64 measurement;
    measurement.data = distanceFilter_->calculate_distance(border_points);
    distance_pub_->publish(measurement);
  }

public:
  std::shared_ptr<ICellingDistanceFilter> distanceFilter_;
  rclcpp::Service<bautiro_ros_interfaces::srv::GetCeilingDistance>::SharedPtr
    service_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_pub_;
};

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  auto approximator = std::make_shared<LinearRegression>();
  auto ceiling_distance_reader =
    std::make_shared<CeilingDistanceFilter>(approximator);

  rclcpp::spin(
    std::make_shared<CeilingDistanceRosNode>(ceiling_distance_reader));

  rclcpp::shutdown();
  return 0;
}
