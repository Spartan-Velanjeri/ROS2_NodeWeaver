// Copyright 2024 Robert Bosch GmbH and its subsidiaries
// All rights reserved, also regarding any disposal, exploitation, reproduction
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#include "linear_regression.hpp"

#include <cmath>
#include <vector>

void LinearRegression::calculate_function_coeff(
  const std::vector<geometry_msgs::msg::Point> & points)
{
  double x_sum = 0.0;
  double y_sum = 0.0;
  double x_squared_sum = 0.0;
  double xy_product_sum = 0.0;
  const auto n = points.size();

  for (const auto & point : points) {
    x_sum += point.x;
    y_sum += point.y;
    x_squared_sum += point.x * point.x;
    xy_product_sum += point.x * point.y;
  }

  slope = (n * xy_product_sum - x_sum * y_sum) /
    (n * x_squared_sum - x_sum * x_sum);
  constant = (y_sum - slope * x_sum) / n;
  double variance_sum = 0.0;
  for (const auto & point : points) {
    const auto y_estim = slope * point.x + constant;
    variance_sum += pow(point.y - y_estim, 2);
  }
  variance = variance_sum / (n - 2);
  std_dev = std::sqrt(variance);
}

double LinearRegression::get_value_from_fcn(double x)
{
  return slope * x + constant;
}
