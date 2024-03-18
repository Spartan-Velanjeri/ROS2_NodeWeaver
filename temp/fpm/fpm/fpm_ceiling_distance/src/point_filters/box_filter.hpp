// Copyright 2024 Robert Bosch GmbH and its subsidiaries
// All rights reserved, also regarding any disposal, exploitation, reproduction
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#ifndef POINT_FILTERS__BOX_FILTER_HPP_
#define POINT_FILTERS__BOX_FILTER_HPP_

#include <vector>

#include "point2d_filter_interface.hpp"

class BoxFilter : public IPoints2dFilter
{
public:
  BoxFilter(double min_x, double max_x, double min_y, double max_y)
  : min_x_(min_x), max_x_(max_x), min_y_(min_y), max_y_(max_y) {}
  ~BoxFilter() override {}
  std::vector<geometry_msgs::msg::Point>
  filter(const std::vector<geometry_msgs::msg::Point> & points) override;

private:
  double min_x_;
  double max_x_;
  double min_y_;
  double max_y_;
};

#endif  // POINT_FILTERS__BOX_FILTER_HPP_
