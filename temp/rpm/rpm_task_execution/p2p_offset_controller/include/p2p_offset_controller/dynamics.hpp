//  Copyright 2023, 2024 Robert Bosch GmbH and its subsidiaries
//
//  All rights reserved, also regarding any disposal, exploitation, reproduction,
//  editing, distribution, as well as in the event of applications for industrial
//  property rights.
#ifndef P2P_OFFSET_CONTROLLER__DYNAMICS_HPP_
#define P2P_OFFSET_CONTROLLER__DYNAMICS_HPP_
#pragma once

namespace p2p_offset_controller
{
typedef struct
{
  double offset_x;
  double offset_y;
  double wheel_separation;
  double wheel_radius;
} state;

typedef struct
{
  double v_x;
  double w_z;
} twist_vel;

typedef struct
{
  double vel_wheel_l;
  double vel_wheel_r;
} joint_param;

typedef struct
{
  double dot_x;
  double dot_y;
  double dot_w;
} cartesian_param;

void offset_dynamics(twist_vel & input, state & state, cartesian_param & output)
{
  output.dot_x = input.v_x + state.offset_y * input.w_z;
  output.dot_y = state.offset_x * input.w_z;
}
void inv_offset_dynamics(cartesian_param & input, state & state, twist_vel & output)
{
  output.v_x = input.dot_x - (state.offset_y * input.dot_y / state.offset_x);
  output.w_z = input.dot_y / state.offset_x;
}

}  // namespace p2p_offset_controller

#endif  // P2P_OFFSET_CONTROLLER__DYNAMICS_HPP_
