// Copyright 2024 Robert Bosch GmbH and its subsidiaries
// All rights reserved, also regarding any disposal, exploitation, reproduction
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#ifndef POINT_FILTERS__POINT2D_FILTER_INTERFACE_HPP_
#define POINT_FILTERS__POINT2D_FILTER_INTERFACE_HPP_

#include <geometry_msgs/msg/point.hpp>

#include <vector>

/**
 * @class 2D points filter interface
 * @brief A class provide an interface which allows to create a filter which takes
 * a vector of 2D points and returns a filtered vector.
 */
class IPoints2dFilter
{
public:
  virtual ~IPoints2dFilter() {}
  /**
    * @brief filter function which takes vector of points and returns filtered vector
    * @param points Vector of points to filter
    * @return filtered vector of points
   */
  virtual std::vector<geometry_msgs::msg::Point>
  filter(const std::vector<geometry_msgs::msg::Point> & points) = 0;
};

#endif  // POINT_FILTERS__POINT2D_FILTER_INTERFACE_HPP_
