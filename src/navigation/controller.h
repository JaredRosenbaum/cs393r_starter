//========================================================================
/*!
\file    controller.h
\brief   Interface for time optimal controller implementation
\author  Daniel Meza, Jared Rosenbaum, Steven Swanbeck, (C) 2024
*/
//========================================================================

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <iostream>
#include <cmath>

class TimeOptimalController {
  public:
    TimeOptimalController(float controller_frequency, float max_speed, float max_acceleration);

    float CalculateSpeed(const float distance_left);

  private:
    float delta_t_; // Time step period

    float max_v_; // Max speed
    float max_a_; // Max acceleration

    float x_;   // Current location
    float v_;   // Current speed

};

#endif  // CONTROLLER_H
