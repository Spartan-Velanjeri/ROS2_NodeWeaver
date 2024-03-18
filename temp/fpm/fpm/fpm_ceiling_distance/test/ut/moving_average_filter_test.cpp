// Copyright 2024 Robert Bosch GmbH and its subsidiaries
// All rights reserved, also regarding any disposal, exploitation, reproduction
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#include <gtest/gtest.h>

#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "point_filters/moving_average_filter.hpp"
#include "test_utils.hpp"

struct MovingAverageTestParams
{
  std::vector<std::vector<geometry_msgs::msg::Point>> scans;
  std::vector<geometry_msgs::msg::Point> expected_output;
  int filter_length;
};

class MovingAverageFilterTest : public ::testing::TestWithParam<MovingAverageTestParams>
{
};

TEST_P(MovingAverageFilterTest, AverageCalculationTest) {
  auto params = GetParam();
  MovingAverageFilter moving_average_filter(params.filter_length);
  std::vector<geometry_msgs::msg::Point> filtered_output;

  for (const auto & scan : params.scans) {
    filtered_output = moving_average_filter.filter(scan);
  }
  EXPECT_EQ(filtered_output, params.expected_output);
}

INSTANTIATE_TEST_SUITE_P(
  MovingAverageFilterTests,
  MovingAverageFilterTest,
  ::testing::Values(
    MovingAverageTestParams{
  {
    {makePoint(1.0, 1.0, 0.0), makePoint(2.0, 2.0, 0.0)},
    {makePoint(3.0, 3.0, 0.0), makePoint(4.0, 4.0, 0.0)}
  },
  {makePoint(2.0, 2.0, 0.0), makePoint(3.0, 3.0, 0.0)},
  2
},
    MovingAverageTestParams{
  {
    {makePoint(7.0, 8.0, 0.0), makePoint(9.0, 8.0, 0.0)},
    {makePoint(1.0, 1.0, 0.0), makePoint(2.0, 2.0, 0.0)},
    {makePoint(3.0, 3.0, 0.0), makePoint(4.0, 4.0, 0.0)},
  },
  {makePoint(2.0, 2.0, 0.0), makePoint(3.0, 3.0, 0.0)},
  2
},
    MovingAverageTestParams{
  {
    {makePoint(1.0, 1.0, 0.0), makePoint(2.0, 2.0, 0.0)},
    {makePoint(3.0, 3.0, 0.0), makePoint(4.0, 4.0, 0.0)}
  },
  {makePoint(3.0, 3.0, 0.0), makePoint(4.0, 4.0, 0.0)},
  1
}
  )
);
