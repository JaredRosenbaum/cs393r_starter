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
#include <queue>
#include <chrono>

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

namespace LatencyCompensation {

// For storing command history within the 
struct LatentCommand {
  float velocity;
  float curvature;
  std::chrono::milliseconds timestamp;
  LatentCommand (float v, float c) : velocity(v), curvature(c) {
    timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
  }
}; // struct LatentData

class LatencyController {

  public:
    LatencyController(float latency, float controller_frequency, float max_speed, float max_acceleration, float max_curvature, float curvature_step, float width, float length, float wheelbase, float margin);
    ~LatencyController();
    void recordCommand(const LatentCommand command);
    void recordCommand(const float v, const float c);

  private:
    float latency_;
    std::queue<LatentCommand> command_history_;
    TimeOptimalController *toc_;

}; // class LatencyController

} // namespace LatencyCompensation

#endif  // CONTROLLER_H
