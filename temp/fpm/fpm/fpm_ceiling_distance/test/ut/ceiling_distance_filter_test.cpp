// Copyright 2024 Robert Bosch GmbH and its subsidiaries
// All rights reserved, also regarding any disposal, exploitation, reproduction
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "approximation/linear_approximation_interface.hpp"
#include "ceiling_distance_filter/ceiling_distance_filter.hpp"
#include "point_filters/point2d_filter_interface.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "test_utils.hpp"

class MockFilter : public IPoints2dFilter
{
public:
  MOCK_METHOD(
    std::vector<geometry_msgs::msg::Point>, filter,
    (const std::vector<geometry_msgs::msg::Point>&), (override));
};

class MockApproximator : public ILinearApproximation
{
public:
  MOCK_METHOD(
    void, calculate_function_coeff,
    (const std::vector<geometry_msgs::msg::Point>&), (override));
  MOCK_METHOD(double, get_value_from_fcn, (double), (override));
};

TEST(CeilingDistanceFilterTest, FilterCalledTest) {
  auto mock_approximator = std::make_shared<MockApproximator>();
  CeilingDistanceFilter filter(mock_approximator);
  sensor_msgs::msg::LaserScan scan;
  scan.ranges = {1.0, 2.0, 3.0, 4.0, 5.0};
  scan.angle_min = 0.0;
  scan.angle_increment = M_PI / 4.0;
  filter.update_measurements(scan);

  auto mock_filter = std::make_shared<MockFilter>();
  filter.add_points_filter(mock_filter);

  EXPECT_CALL(*mock_filter, filter(::testing::_))
  .Times(1)
  .WillOnce(::testing::Return(std::vector<geometry_msgs::msg::Point>()));

  filter.calculate_distance(std::vector<geometry_msgs::msg::PointStamped>());
}
