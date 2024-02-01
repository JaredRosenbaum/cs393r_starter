//========================================================================
/*!
\file    controller.cc
\brief   Interface for time optimal controller implementation
\author  Daniel Meza, Jared Rosenbaum, Steven Swanbeck, (C) 2024
*/
//========================================================================

#include "controller.h"

TimeOptimalController::TimeOptimalController(float time_step, float max_speed, float max_acceleration, float max_curvature, float curvature_step, float width, float length, float wheelbase, float margin) :
delta_t_(time_step),
v_max_(max_speed), a_max_(max_acceleration),
curv_max_(max_curvature), curv_step_(curvature_step),
b_(wheelbase), m_(margin),
w_(width), l_(length) {
  x_ = 0;   // car begins at 0;
  v_ = 0;   // car begins at rest
}

float TimeOptimalController::CalculateSpeed(const float free_path_length) {
  // TODO Someone double check my "free_path_length > ..." calculations

  // Accelerate Case
  // Use kinematic equations to check if we have enough distance to accelerate one more time-step
  if (v_ < v_max_ && free_path_length > 
  ((v_ * delta_t_) + ((v_ + a_max_ * pow(delta_t_, 2)) / 2 ) + (pow(v_ + a_max_ * delta_t_, 2) / 2 * a_max_))) {
    v_ = v_ + a_max_ * delta_t_;  // speed increases by acceleration rate
  }
  // Cruise Case
  // Use kinematic equations to check if we have enough distance to cruise one more time-step
  else if (v_ == v_max_ && free_path_length > 
  ((v_ + delta_t_) + (pow(v_, 2) / 2 * a_max_))) {
    ; // speed remains the same
  }
  // Decelerate Case
  // Not enough distance, calculate deceleration rate
  else {
    float d = pow(v_, 2) / (2 * free_path_length);
    v_ = v_ - d * delta_t_;  // speed decreases by acceleration rate
  }

  return v_;
}

void TimeOptimalController::EvaluatePaths(const std::vector<Vector2f>& point_cloud) {
  float free_path;
  float clearance;
  float goal_distance;
  float sel_free_path;
  float sel_curv;
  float max_score = -1.0;

  // Evaluate all possible paths and select optimal option
  // TODO: Iterate correctly
  for (float curv = -curv_max_; curv <= curv_max_; curv += curv_step_) {
    // Calculate free path lenght, clearance, and distance to goal
    free_path = CalculateFreePathLength(point_cloud, curv);
    // clearance = CalculateClearance();
    // goal_distance = CalculateDistanceToGoal();

    // Calculate score and update selection
    // float score = free_path + w1_ * clearance + w2_ * goal_distance;
    // if (score > max_score) {
    //   max_score = score;
    //   sel_free_path = free_path;
    //   sel_curv = curv;
    // }
  }

  // TODO return selected free path length and curvature
}

float TimeOptimalController::CalculateFreePathLength(const std::vector<Vector2f>& point_cloud, float curvature) {
  float fpl_ = 9.9; //todo experiment
  Vector2f p(0.0, 0.0);

  // Moving in a straight line
  if (abs(curvature) < 0.01) {
    // Loop through point cloud
    for (int i = 0; i < (int)point_cloud.size(); i++) {
      p = point_cloud[i];

      // Update minimum free path length for lasers in front of car
      if ((abs(p.y()) < w_ / 2 + m_) && p.x() < fpl_) {
        fpl_ = p.x();
      }
    } // At this point, d equals the minimum distance to an obstacle directly in front of the car. It does not account for car length.

    // Adjust d for car length and margin
    fpl_ -= (l_+b_)/2+m_;
  }
  // Moving along an arc
  else {
    float radius = 1.0/curvature;
    // Loop through point cloud
    for (int i = 0; i < (int)point_cloud.size(); i++) {
      p = point_cloud[i];
      //Check which one of the toruses the point lies within, if any
      float r_p = sqrt(pow(p.x(),2)+pow((radius-p.y()),2));
      float theta = atan2(p.x(),(radius-p.y()));
      // Condition one: The point hits the inner side of the car.
      // .Theta > 0 
      if ((radius-w_/2-m_) <= r_p < sqrt(pow(radius-w_/2-m_,2)+pow((l_+b_)/2+m_,2))){
        float psi = acos((radius-w_/2-m_)/r_p);
        float phi = theta - psi;
        if (radius*phi < fpl_){
          fpl_ = radius * phi;
        }
      }
      // Condition two: The point hits the front of the car
      //. Theta > 0
      if (sqrt(pow(radius-w_/2-m_,2)+pow((l_+b_)/2+m_,2)) <= r_p <= sqrt(pow(radius+w_/2+m_,2)+pow((l_+b_)/2+m_,2))){
        float psi = asin(((l_+b_)/2+m_)/r_p);
        float phi = theta - psi;
        if (radius*phi < fpl_){
          fpl_ = radius * phi;
        }
      }
      // Condition three: The point hits the outer rear side of the car. 
      //. Theta < 0 
      if ((radius+w_/2 <= r_p <= sqrt(pow(radius+w_/2+m_,2)+pow((l_-b_)/2+m_,2)))){
        float psi = asin(p.x()/r_p);
        float phi = theta - psi;
        if (radius*phi < fpl_){
          fpl_ = radius * phi;
        }
      }
    }
  }
  return fpl_;
}

float TimeOptimalController::CalculateClearance(float curvature){
  Vector2f p(0.0, 0.0);

  // Moving in a straight line
  if (abs(curvature) < 0.01) {
  }
}

float TimeOptimalController::CalculateDistanceToGoal(){

  
}
