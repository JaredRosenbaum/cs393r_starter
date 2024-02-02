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
    TimeOptimalController(float controller_frequency, float max_speed, float max_acceleration, float max_curvature, float max_clearance, float curvature_step, float width, float length, float wheelbase, float margin);

    float CalculateSpeed(const float distance_left);

    float CalculateClearance(const std::vector<Vector2f>& point_cloud, float curvature, float fpl);

    float CalculateDistanceToGoal();

    void EvaluatePaths(const std::vector<Vector2f>& point_cloud); 

    float CalculateFreePathLength(const std::vector<Vector2f>& point_cloud, float curvature);

  private:
    float delta_t_;     // Time step period
    float v_max_;       // Max speed
    float a_max_;       // Max acceleration
    float curv_max_;    // Max curvature
    float curv_step_;   // Curvature step
    float c_max_;       // Max Clearance
    float b_;        // Wheel base
    float m_;      // Margin
    float w_;       // Width
    float l_;      // Length

    float x_;   // Current location
    float v_;   // Current speed

};

// struct LatentCommands {
//   Eigen::Vector2f velocity;
//   Eigen::Vector2f curvature;

// }; // struct LatentData

// class LatencyController {

//   public:

//   private:
//     queue
//     TimeOptimalController *toc_;

// }; // class LatencyController

#endif  // CONTROLLER_H
