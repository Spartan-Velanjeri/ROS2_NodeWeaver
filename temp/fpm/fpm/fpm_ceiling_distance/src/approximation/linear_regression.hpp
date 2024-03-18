// Copyright 2024 Robert Bosch GmbH and its subsidiaries
// All rights reserved, also regarding any disposal, exploitation, reproduction
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#ifndef APPROXIMATION__LINEAR_REGRESSION_HPP_
#define APPROXIMATION__LINEAR_REGRESSION_HPP_

#include <geometry_msgs/msg/point.hpp>

#include <vector>

#include "linear_approximation_interface.hpp"

class LinearRegression : public ILinearApproximation
{
public:
  virtual ~LinearRegression() {}
  void calculate_function_coeff(const std::vector<geometry_msgs::msg::Point> & points) override;
  double get_value_from_fcn(double x) override;

private:
  double slope;
  double constant;
};

#endif  // APPROXIMATION__LINEAR_REGRESSION_HPP_
