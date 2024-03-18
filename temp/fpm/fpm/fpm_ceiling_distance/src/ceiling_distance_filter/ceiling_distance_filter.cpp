// Copyright 2024 Robert Bosch GmbH and its subsidiaries
// All rights reserved, also regarding any disposal, exploitation, reproduction
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#include "ceiling_distance_filter.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

double CeilingDistanceFilter::calculate_distance(
  const std::vector<geometry_msgs::msg::PointStamped> & border_points)
{
  if (!last_msg_.has_value()) {
    return 0;
  }

  std::vector<geometry_msgs::msg::Point> points;
  for (size_t i = 0; i < last_msg_->ranges.size(); ++i) {
    points.push_back(get_point_coordinates(*last_msg_, i));
  }
  for (const auto & filter : point2d_filters_) {
    points = filter->filter(points);
  }

  if (border_points.size() != 2) {
    return 0;
  }
  // we are interested only in that part of measurements
  points.erase(
    std::remove_if(
      points.begin(), points.end(),
      [&](const auto & point) {
        return point.y < std::min(
          border_points.at(0).point.y,
          border_points.at(1).point.y) ||
        point.y > std::max(
          border_points.at(0).point.y,
          border_points.at(1).point.y);
      }),
    points.end());

  for (auto & point : points) {
    const auto tmp_x = point.x;
    point.x = point.y;
    point.y = tmp_x;
  }
  approximator_->calculate_function_coeff(points);
  if (approximator_->get_std_dev() > 0.02) {
    return 0;
  } else {
    return std::min(
      approximator_->get_value_from_fcn(border_points.at(0).point.y),
      approximator_->get_value_from_fcn(border_points.at(1).point.y));
  }

  return 0;
}

void CeilingDistanceFilter::update_measurements(
  const sensor_msgs::msg::LaserScan & scan)
{
  last_msg_ = scan;
}

geometry_msgs::msg::Point CeilingDistanceFilter::get_point_coordinates(
  const sensor_msgs::msg::LaserScan & scan, int index)
{
  geometry_msgs::msg::Point point;
  point.x = scan.ranges.at(index) *
    cos(scan.angle_min + scan.angle_increment * index);
  point.y = scan.ranges.at(index) *
    sin(scan.angle_min + scan.angle_increment * index);
  point.z = 0;
  return point;
}

void CeilingDistanceFilter::add_points_filter(
  std::shared_ptr<IPoints2dFilter> filter)
{
  point2d_filters_.push_back(filter);
}
