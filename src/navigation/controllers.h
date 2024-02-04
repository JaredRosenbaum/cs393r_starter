//========================================================================
/*!
\file    controller.h
\brief   Interface for time optimal controller implementation
\author  Daniel Meza, Jared Rosenbaum, Steven Swanbeck, (C) 2024
*/
//========================================================================

#pragma once

#include <iostream>
#include <cmath>
#include <vector>
#include <queue>
#include <chrono>
#include "eigen3/Eigen/Dense"

#include "vehicles.hpp"
#include "functions.h"

using Eigen::Vector2f;

namespace controllers {

struct PathCandidate {
  float curvature;
  float free_path_length;
  float score;
  float clearance;
  float goal_distance;
  PathCandidate(){}
  PathCandidate(float s) : score(s) {}
  PathCandidate(float c, float fpl) : curvature(c), free_path_length(fpl) {};
}; // struct PathCandidate

namespace time_optimal_1D {

struct ControlCommand {
  float velocity;
  float curvature;
  ControlCommand(float v, float c) : velocity(v), curvature(c) {}
}; // struct ControlCommand

class Controller {
  public:
    Controller(vehicles::Car *car, float control_interval, float margin, float max_clearance, float curvature_sampling_interval);
    // TimeOptimalController(float controller_frequency, float max_speed, float max_acceleration, float max_curvature, float max_clearance, float curvature_step, float width, float length, float wheelbase, float track_width, float margin);

    float calculateControlSpeed(const float current_speed, const float distance_left);

    float calculateClearance(const std::vector<Vector2f>& point_cloud, const float curvature, const float free_path_length);

    float calculateDistanceToGoal();

    PathCandidate evaluatePaths(const std::vector<Vector2f>& point_cloud); 
    
    // void selectPath(const std::vector<Vector2f>& point_cloud); 

    float calculateFreePathLength(const std::vector<Vector2f>& point_cloud, float curvature);

    ControlCommand generateCommand(const std::vector<Vector2f>& point_cloud, const float current_speed);

  private:
    vehicles::Car *car_;
    float control_interval_;    // Time step period
    float margin_;              // Margin of error around car
    float max_clearance_;       // TODO
    float curvature_sampling_interval_; // Dicretization of curvature paths
};

} // namespace time_optimal_1D


namespace latency_compensation {

// For storing command history within the latency compensator
struct LatentCommand {
  time_optimal_1D::ControlCommand command;
  std::chrono::milliseconds timestamp;
  LatentCommand (float v, float c) : command(v, c) {
    timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
  }
}; // struct LatentCommand

class Controller {

  public:
    Controller(vehicles::Car *car, float control_interval, float margin, float max_clearance, float curvature_sampling_interval, float latency);
    ~Controller();
    void recordCommand(const LatentCommand command);
    void recordCommand(const float v, const float c);

  private:
    vehicles::Car *car_;
    float latency_;
    time_optimal_1D::Controller *toc_;
    std::queue<LatentCommand> command_history_;

}; // class LatencyController

} // namespace latency_compensation

} // namespace controllers
