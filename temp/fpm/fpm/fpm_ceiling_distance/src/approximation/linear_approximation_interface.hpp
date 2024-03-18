// Copyright 2024 Robert Bosch GmbH and its subsidiaries
// All rights reserved, also regarding any disposal, exploitation, reproduction
// editing, distribution, as well as in the event of applications for industrial
// property rights.

#ifndef APPROXIMATION__LINEAR_APPROXIMATION_INTERFACE_HPP_
#define APPROXIMATION__LINEAR_APPROXIMATION_INTERFACE_HPP_

#include <geometry_msgs/msg/point.hpp>

#include <vector>

/**
 * @class Linear approximation interface
 * @brief A class provide an interface which allows to calculate linear function
 * coefficients as well as get value of the function for a given x. It also returns
 * vatiance and standard derivative of the approximation to evaluate approximation quality.
 */
class ILinearApproximation
{
public:
  virtual ~ILinearApproximation() {}
  /**
   * @brief Calculates approximation function coefficients and stores them internally
   * @param points Vector of points to calculate approximation for
  */
  virtual void calculate_function_coeff(const std::vector<geometry_msgs::msg::Point> & points) = 0;

  /**
   * @brief Get value of a linear function for a given x
   * @param x Value to calculate function for
   * @return Value of the function for a given x
  */
  virtual double get_value_from_fcn(double x) = 0;

  /**
   * @brief Get variance of the approximation
   * @return variance of the approximation
  */
  double get_variance()
  {
    return variance;
  }

  /**
   * @brief Get standard deviation of the approximation
   * @return standard deviation of the approximation
  */
  double get_std_dev()
  {
    return std_dev;
  }

protected:
  double variance;
  double std_dev;
};

#endif  // APPROXIMATION__LINEAR_APPROXIMATION_INTERFACE_HPP_
