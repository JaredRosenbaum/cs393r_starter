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
#include <deque>
#include <chrono>
#include "eigen3/Eigen/Dense"
#include "ros/ros.h"

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

struct State2D {
  Eigen::Vector2f position;
  double theta;
  float speed;
}; // struct State2D

namespace time_optimal_1D {

struct Command {
  float velocity;
  float curvature;
  Command(float v, float c) : velocity(v), curvature(c) {}
}; // struct Command

class Controller {
  public:
    Controller(vehicles::Car *car, float control_interval, float margin, float max_clearance, float curvature_sampling_interval);
    // TimeOptimalController(float controller_frequency, float max_speed, float max_acceleration, float max_curvature, float max_clearance, float curvature_step, float width, float length, float wheelbase, float track_width, float margin);

    float calculateControlSpeed(const float current_speed, const float distance_left);

    float calculateClearance(const std::vector<Vector2f>& point_cloud, const float curvature, const float free_path_length);

    float calculateDistanceToGoal(const float curvature);

    PathCandidate evaluatePaths(const std::vector<Vector2f>& point_cloud); 
    
    // void selectPath(const std::vector<Vector2f>& point_cloud); 

    float calculateFreePathLength(const std::vector<Vector2f>& point_cloud, const float curvature);

    Command generateCommand(const std::vector<Vector2f>& point_cloud, const float current_speed);

    float getControlInterval();

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
struct CommandStamped {
  time_optimal_1D::Command command;
  // std::chrono::milliseconds timestamp;
  double timestamp;
  CommandStamped (time_optimal_1D::Command com) : command(com) {
    timestamp = ros::Time::now().toSec();
  }
}; // struct CommandStamped

class Controller {

  public:
    Controller(vehicles::Car *car, float control_interval, float margin, float max_clearance, float curvature_sampling_interval, float latency);
    ~Controller();

    time_optimal_1D::Command generateCommand(const std::vector<Vector2f>& point_cloud, const float current_speed, const double last_data_timestamp);

    void recordCommand(const time_optimal_1D::Command command);
  private:
    vehicles::Car *car_;

    State2D projectState(const float current_speed, const double last_msg_timestamp);

    std::vector<Eigen::Vector2f> transformCloud (std::vector<Eigen::Vector2f> cloud, const State2D &state);

    void recordCommand(const CommandStamped command);

    void printState(const State2D &state);

    float calculateFreePathLength(const std::vector<Vector2f>& point_cloud, const float curvature, const double last_data_timestamp);


    float latency_;
    std::deque<CommandStamped> command_history_;
    time_optimal_1D::Controller *toc_;

    // double last_msg_timestamp_;

}; // class LatencyController

} // namespace latency_compensation

} // namespace controllers
