// Copyright 2024 Robert Bosch GmbH and its subsidiaries
// All rights reserved, also regarding any disposal, exploitation, reproduction
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#include "moving_average_filter.hpp"

#include <vector>

#include "point2d_filter_interface.hpp"

std::vector<geometry_msgs::msg::Point> MovingAverageFilter::filter(
  const std::vector<geometry_msgs::msg::Point> & points)
{
  previous_measures_.push_back(points);
  if (previous_measures_.size() > window_size_) {
    previous_measures_.pop_front();
  }

  std::vector<geometry_msgs::msg::Point> filtered_points;
  for (size_t i = 0; i < points.size(); ++i) {
    double x = 0;
    double y = 0;
    for (size_t j = 0; j < previous_measures_.size(); ++j) {
      x += previous_measures_.at(j).at(i).x;
      y += previous_measures_.at(j).at(i).y;
    }
    x /= previous_measures_.size();
    y /= previous_measures_.size();
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    filtered_points.push_back(p);
  }

  return filtered_points;
}
