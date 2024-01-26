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
#include <vector>

#include "eigen3/Eigen/Dense"

using Eigen::Vector2f;

class TimeOptimalController {
  public:
    TimeOptimalController(float controller_frequency, float max_speed, float max_acceleration, float max_curvature, float curvature_step, float width);

    float CalculateSpeed(const float distance_left);

    void EvaluatePaths(const std::vector<Vector2f>& point_cloud);

    void CalculateFreePathLength(const std::vector<Vector2f>& point_cloud, float theta);

  private:
    float delta_t_;     // Time step period
    float v_max_;       // Max speed
    float a_max_;       // Max acceleration
    float curv_max_;    // Max curvature
    float curv_step_;   // Curvature step
    float width_;       // Total width considered for obstacle detection
    float length_;      // Total length considered for obstacle detection

    float x_;   // Current location
    float v_;   // Current speed

};

#endif  // CONTROLLER_H
