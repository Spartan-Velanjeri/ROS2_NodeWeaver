// Copyright 2024 Robert Bosch GmbH and its subsidiaries
// All rights reserved, also regarding any disposal, exploitation, reproduction
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#ifndef CEILING_DISTANCE_FILTER__CEILING_DISTANCE_FILTER_HPP_
#define CEILING_DISTANCE_FILTER__CEILING_DISTANCE_FILTER_HPP_

#include <optional>
#include <memory>
#include <vector>

#include "../approximation/linear_regression.hpp"
#include "../point_filters/point2d_filter_interface.hpp"
#include "ceiling_distance_filter_interface.hpp"

class CeilingDistanceFilter : public ICellingDistanceFilter
{
public:
  explicit CeilingDistanceFilter(
    std::shared_ptr<ILinearApproximation> approximator)
  : approximator_(approximator) {}
  ~CeilingDistanceFilter() {}

  double calculate_distance(
    const std::vector<geometry_msgs::msg::PointStamped>
    & border_points) override;
  void update_measurements(const sensor_msgs::msg::LaserScan & scan) override;
  void add_points_filter(std::shared_ptr<IPoints2dFilter> filter);

private:
  geometry_msgs::msg::Point
  get_point_coordinates(const sensor_msgs::msg::LaserScan & scan, int index);

  std::optional<sensor_msgs::msg::LaserScan> last_msg_;
  std::vector<std::shared_ptr<IPoints2dFilter>> point2d_filters_;
  std::shared_ptr<ILinearApproximation> approximator_;
};

#endif  // CEILING_DISTANCE_FILTER__CEILING_DISTANCE_FILTER_HPP_
