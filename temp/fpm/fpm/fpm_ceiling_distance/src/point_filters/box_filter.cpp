// Copyright 2024 Robert Bosch GmbH and its subsidiaries
// All rights reserved, also regarding any disposal, exploitation, reproduction
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#include "box_filter.hpp"

#include <vector>

#include "point2d_filter_interface.hpp"

std::vector<geometry_msgs::msg::Point>
BoxFilter::filter(const std::vector<geometry_msgs::msg::Point> & points)
{
  std::vector<geometry_msgs::msg::Point> filtered_points;
  for (auto point : points) {
    if (point.x > min_x_ && point.x < max_x_ && point.y > min_y_ &&
      point.y < max_y_)
    {
      filtered_points.push_back(point);
    }
  }
  return filtered_points;
}
