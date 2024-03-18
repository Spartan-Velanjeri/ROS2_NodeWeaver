// Copyright 2024 Robert Bosch GmbH and its subsidiaries
// All rights reserved, also regarding any disposal, exploitation, reproduction
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#include <gtest/gtest.h>

#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "point_filters/box_filter.hpp"
#include "test_utils.hpp"

struct BoxTestParams
{
  geometry_msgs::msg::Point point;
  bool expected_inside;
  double box_width;
  double box_height;
};

class BoxFilterTest : public ::testing::TestWithParam<BoxTestParams>
{
};

TEST_P(BoxFilterTest, PointInOrOutSquare) {
  auto params = GetParam();

  BoxFilter box_filter(0.0, params.box_width, 0.0, params.box_height);
  std::vector<geometry_msgs::msg::Point> points = {params.point};
  auto filtered_points = box_filter.filter(points);

  if (params.expected_inside) {
    ASSERT_EQ(filtered_points.size(), (size_t)1);
    EXPECT_DOUBLE_EQ(filtered_points.at(0).x, params.point.x);
    EXPECT_DOUBLE_EQ(filtered_points.at(0).y, params.point.y);
  } else {
    ASSERT_EQ(filtered_points.size(), (size_t)0);
  }
}

INSTANTIATE_TEST_SUITE_P(
  BoxFilterTests,
  BoxFilterTest,
  ::testing::Values(
    BoxTestParams{makePoint(0.5, 0.5, 0.0), true, 1.0, 1.0},  // Inside the box
    BoxTestParams{makePoint(1.5, 0.5, 0.0), false, 1.0, 1.0},  // Outside the box
    BoxTestParams{makePoint(0.5, 1.5, 0.0), false, 1.0, 1.0},  // Outside the box
    BoxTestParams{makePoint(1.5, 1.5, 0.0), true, 2.0, 2.0}  // Inside the box
  )
);
