// Copyright 2024 Robert Bosch GmbH and its subsidiaries
// All rights reserved, also regarding any disposal, exploitation, reproduction
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#include <gtest/gtest.h>

#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "approximation/linear_regression.hpp"
#include "test_utils.hpp"


struct LinearRegressionTestParams
{
  std::vector<geometry_msgs::msg::Point> points;
  geometry_msgs::msg::Point expected_point_in_fcn;
  double expected_variance;
  double expected_std_dev;
};

class LinearRegressionTest : public ::testing::TestWithParam<LinearRegressionTestParams>
{
};

TEST_P(LinearRegressionTest, CalculateFunctionCoeffTest) {
  auto params = GetParam();
  LinearRegression lr;

  lr.calculate_function_coeff(params.points);
  const auto point_in_fcn_y = lr.get_value_from_fcn(params.expected_point_in_fcn.x);

  EXPECT_NEAR(point_in_fcn_y, params.expected_point_in_fcn.y, 1e-3);
  EXPECT_NEAR(lr.get_variance(), params.expected_variance, 1e-3);
  EXPECT_NEAR(lr.get_std_dev(), params.expected_std_dev, 1e-3);
}

INSTANTIATE_TEST_SUITE_P(
  LinearRegressionTests,
  LinearRegressionTest,
  ::testing::Values(
    LinearRegressionTestParams{
  {makePoint(1.0, 1.0, 0.0), makePoint(2.0, 2.0, 0.0), makePoint(3.0, 3.0, 0.0)},
  makePoint(4.0, 4.0, 0.0),
  0.0, 0.0
},
    LinearRegressionTestParams{
  {makePoint(1.0, 1.0, 0.0), makePoint(2.0, 2.0, 0.0), makePoint(3.0, 4.0, 0.0)},
  makePoint(4.0, 5.333, 0.0),
  0.166, 0.408
},
    LinearRegressionTestParams{
  {makePoint(1.0, 1.0, 0.0), makePoint(5.0, -5.0, 0.0), makePoint(10.0, -9.0, 0.0)},
  makePoint(4.0, -2.869, 0.0),
  1.607, 1.268
}
  )
);
