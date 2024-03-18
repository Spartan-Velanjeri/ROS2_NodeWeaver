// Copyright 2024 Robert Bosch GmbH and its subsidiaries
// All rights reserved, also regarding any disposal, exploitation, reproduction
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#ifndef POINT_FILTERS__MOVING_AVERAGE_FILTER_HPP_
#define POINT_FILTERS__MOVING_AVERAGE_FILTER_HPP_

#include <deque>
#include <vector>

#include "point2d_filter_interface.hpp"

class MovingAverageFilter : public IPoints2dFilter
{
public:
  explicit MovingAverageFilter(int window_size)
  : window_size_(window_size) {}
  ~MovingAverageFilter() override {}

  std::vector<geometry_msgs::msg::Point>
  filter(const std::vector<geometry_msgs::msg::Point> & points);

private:
  size_t window_size_;
  std::deque<std::vector<geometry_msgs::msg::Point>> previous_measures_;
};

#endif  // POINT_FILTERS__MOVING_AVERAGE_FILTER_HPP_
