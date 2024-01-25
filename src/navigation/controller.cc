//========================================================================
/*!
\file    controller.cc
\brief   Interface for time optimal controller implementation
\author  Daniel Meza, Jared Rosenbaum, Steven Swanbeck, (C) 2024
*/
//========================================================================

#include "controller.h"

TimeOptimalController::TimeOptimalController(float time_step, float max_speed, float max_acceleration) :
delta_t_(time_step),
max_v_(max_speed), 
max_a_(max_acceleration) {
  x_ = 0;   // car begins at 0;
  v_ = 0;   // car begins at rest
}


float TimeOptimalController::CalculateSpeed(const float distance_left) {
  // TODO Someone double check my "distance_left > ..." calculations

  // Accelerate Case
  // Use kinematic equations to check if we have enough distance to accelerate one more time-step
  if (v_ < max_v_ && distance_left > 
  ((v_ * delta_t_) + ((v_ + max_a_ * pow(delta_t_, 2)) / 2 ) + (pow(v_ + max_a_ * delta_t_, 2) / 2 * max_a_))) {
    v_ = v_ + max_a_ * delta_t_;  // speed increases by acceleration rate
  }
  // Cruise Case
  // Use kinematic equations to check if we have enough distance to cruise one more time-step
  else if (v_ == max_v_ && distance_left > 
  ((v_ + delta_t_) + (pow(v_, 2) / 2 * max_a_))) {
    ; // speed remains the same
  }
  // Decelerate Case
  // Not enough distance, calculate deceleration rate
  else {
    float d = pow(v_, 2) / (2 * distance_left);
    v_ = v_ - d * delta_t_;  // speed decreases by acceleration rate
  }

  return v_;
}
