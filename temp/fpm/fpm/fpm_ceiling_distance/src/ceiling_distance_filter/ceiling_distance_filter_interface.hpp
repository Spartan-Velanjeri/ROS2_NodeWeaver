// Copyright 2024 Robert Bosch GmbH and its subsidiaries
// All rights reserved, also regarding any disposal, exploitation, reproduction
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#ifndef CEILING_DISTANCE_FILTER__CEILING_DISTANCE_FILTER_INTERFACE_HPP_
#define CEILING_DISTANCE_FILTER__CEILING_DISTANCE_FILTER_INTERFACE_HPP_

#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <vector>

/**
 * @class Ceiling distance filter interface
 * @brief A class provide an interface to calculate a distance to the closest point
 * to the ceiling within given range
 */
class ICellingDistanceFilter
{
public:
  /**
    * @brief Function calculates distance to the closest point to the ceiling within given range
    * @param border_points Border points of beetween which the distance is considered
    * @return Distance to the closest point or 0 if measure is invalid
   */
  virtual double calculate_distance(
    const std::vector<geometry_msgs::msg::PointStamped> & border_points) = 0;

  /**
    * @brief Function updates measurements based on laser scan message
    * @param scan Laser scan message
   */
  virtual void update_measurements(const sensor_msgs::msg::LaserScan & scan) = 0;
  virtual ~ICellingDistanceFilter() {}
};

#endif  // CEILING_DISTANCE_FILTER__CEILING_DISTANCE_FILTER_INTERFACE_HPP_
