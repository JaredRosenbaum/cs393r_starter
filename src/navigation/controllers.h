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
#include "shared/math/math_util.h"

#include "vehicles.hpp"
#include "functions.h"

using Eigen::Vector2f;
using math_util::Sq;
using math_util::Sign;
using math_util::Pow;

namespace controllers {

struct PathCandidate {
  float curvature;
  float free_path_length;
  float score;
  float clearance;
  float goal_distance;
  float deviance;
  PathCandidate(){}
  PathCandidate(float s) : score(s) {}
  PathCandidate(float c, float fpl) : curvature(c), free_path_length(fpl) {};
  bool operator < (const PathCandidate &pc) const
  {
    return (score > pc.score);
  }
  PathCandidate(const std::vector<PathCandidate> &paths) {
    curvature = 0;
    free_path_length = 0;
    clearance = 0;
    goal_distance = 0;
    deviance = 0;
    score = 0;
    for (const auto &path : paths) {
      curvature += path.curvature;
      free_path_length += path.free_path_length;
      clearance += path.clearance;
      goal_distance += path.goal_distance;
      deviance += path.deviance;
    }
    curvature /= static_cast<int>(paths.size());
    free_path_length /= static_cast<int>(paths.size());
    clearance /= static_cast<int>(paths.size());
    goal_distance /= static_cast<int>(paths.size());
    deviance /= static_cast<int>(paths.size());
  };
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

    float calculateControlSpeed(float current_speed, const float distance_left);

    // float calculateClearance(const std::vector<Vector2f>& point_cloud, const float curvature, const float free_path_length);
    void calculateClearance(const std::vector<Vector2f>& point_cloud, PathCandidate &path);

    void computePathOption(PathCandidate &path_option, float curvature, const std::vector<Eigen::Vector2f>& point_cloud);

    float calculateDistanceToGoal(const float curvature);

    PathCandidate evaluatePaths(const std::vector<Vector2f>& point_cloud, std::vector<PathCandidate> &candidates); 

    float calculateFreePathLength(const std::vector<Vector2f>& point_cloud, const float curvature);

    Command generateCommand(const std::vector<Vector2f>& point_cloud, const float current_speed, std::vector<PathCandidate> &path_candidates, PathCandidate &best_path);

    float getControlInterval();

  private:
    vehicles::Car *car_;
    float control_interval_;    // Time step period
    float margin_;              // Margin of error around car
    float max_clearance_;       // Max clearance value assigned to paths without observed obstacles
    float curvature_sampling_interval_; // Dicretization of curvature paths

    float previous_curvature_; // used to pick paths more similar to previously generated ones
};

} // namespace time_optimal_1D


namespace latency_compensation {

// For storing command history within the latency compensator
struct CommandStamped {
  time_optimal_1D::Command command;
  double timestamp;
  CommandStamped (time_optimal_1D::Command com) : command(com) {
    timestamp = ros::Time::now().toSec();
  }
}; // struct CommandStamped

class Controller {

  public:
    Controller(vehicles::Car *car, float control_interval, float margin, float max_clearance, float curvature_sampling_interval, float latency);
    ~Controller();

    time_optimal_1D::Command generateCommand(const std::vector<Vector2f>& point_cloud, const float current_speed, const double last_data_timestamp, std::vector<PathCandidate> &path_candidates, PathCandidate &best_path);

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

}; // class Controller

} // namespace latency_compensation

} // namespace controllers
