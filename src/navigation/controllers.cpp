//========================================================================
/*!
\file    controllers.ccpp
\brief   Interface for time optimal controller implementation
\author  Daniel Meza, Jared Rosenbaum, Steven Swanbeck, (C) 2024
*/
//========================================================================

#include "controllers.h"

namespace controllers {

// -------------------------------------------------------------------------
// * 1D TIME-OPTIMAL CONTROL
// -------------------------------------------------------------------------

namespace time_optimal_1D {

Controller::Controller(vehicles::Car *car, float control_interval, float margin, float max_clearance, float curvature_sampling_interval) 
: car_(car), control_interval_(control_interval), max_clearance_(max_clearance), curvature_sampling_interval_(curvature_sampling_interval) 
{}

float Controller::calculateControlSpeed(const float current_speed, const float free_path_length)
{
  float control_speed {0.0};

  // Accelerate Case
  // Use kinematic equations to check if we have enough distance to accelerate one more time-step
  if (current_speed < car_->limits_.max_speed_ && free_path_length > 
  ((current_speed * control_interval_) + ((current_speed + car_->limits_.max_acceleration_ * pow(control_interval_, 2)) / 2 ) + (pow(current_speed + car_->limits_.max_acceleration_ * control_interval_, 2) / 2 * car_->limits_.max_acceleration_))) {
    control_speed = current_speed + car_->limits_.max_acceleration_ * control_interval_;  // speed increases by acceleration rate
  }
  // Cruise Case
  // Use kinematic equations to check if we have enough distance to cruise one more time-step
  else if (current_speed == car_->limits_.max_speed_ && free_path_length > 
  ((current_speed * control_interval_) + (pow(current_speed, 2) / 2 * car_->limits_.max_acceleration_))) {
    control_speed = current_speed; // speed remains the same
  }
  // Decelerate Case
  // Not enough distance, calculate deceleration rate
  else {
    // TODO Remove since we are using a slam on the brakes approach
    // float d = pow(current_speed, 2) / (2 * free_path_length);
    // if ((d < 0) || (std::abs(d) > std::abs(car_->limits_.max_acceleration_))) {
    //   d = car_->limits_.max_acceleration_;
    // }
    // control_speed = current_speed - d * control_interval_;  // speed decreases by acceleration rate
    control_speed = current_speed - car_->limits_.max_acceleration_ * control_interval_;  // speed decreases by acceleration rate
    if (control_speed < 0)  // prevent reversal
      control_speed = 0;
  }

  return control_speed;
}

float Controller::calculateFreePathLength(const std::vector<Vector2f>& point_cloud, float curvature)
{
  float free_path_length {10.0f - (margin_ + (car_->dimensions_.length_ + car_->dimensions_.wheelbase_) / 2)}; // TODO set this properly
  Vector2f point(0.0, 0.0);

  if (std::abs(curvature) < 0.01) { // Straight line case
    // Loop through point cloud
    for (int i = 0; i < (int)point_cloud.size(); i++) {
      point = point_cloud[i];

      // Update minimum free path length for lasers in front of car
      // only consider points in front of the car
      if (std::abs(point.y()) < (car_->dimensions_.width_ / 2 + margin_) && point.x() > 0) {

        // calculate candidate free path length
        float candidate_free_path_length {point.x() - (margin_ + (car_->dimensions_.length_ + car_->dimensions_.wheelbase_) / 2)};

        // see if the candidate is shorter than the actual, update if so
        if (candidate_free_path_length < free_path_length) {
          free_path_length = candidate_free_path_length;
        }
      }
    }
  } else { // Moving along an arc
    float radius {1.0f / curvature};

    // Handle right turns by symmetry
    if (curvature < 0) {
      radius *= -1;
    }

    // calculating values that will be useful so we don't have to calculate them each iteration
    float inside_rear_axle_radius {radius - (margin_ + car_->dimensions_.width_ / 2)};
    float inside_front_corner_radius {(float)sqrt(pow(radius - (margin_ + car_->dimensions_.width_ / 2), 2) + pow(margin_ + (car_->dimensions_.length_ + car_->dimensions_.wheelbase_) / 2, 2))};
    float outside_front_corner_radius {(float)sqrt(pow(radius + (margin_ + car_->dimensions_.width_ / 2), 2) + pow(margin_ + (car_->dimensions_.length_ + car_->dimensions_.wheelbase_) / 2, 2))};
    float outside_rear_corner_radius {(float)sqrt(pow(radius + (margin_ + car_->dimensions_.width_ / 2), 2) + pow(margin_ + (car_->dimensions_.length_ - car_->dimensions_.wheelbase_) / 2, 2))};
    // float outside_rear_axle_radius {radius + (margin_ + car_->dimensions_.width_ / 2)};

    // Loop through point cloud
    for (int i = 0; i < (int)point_cloud.size(); i++) {
      point = point_cloud[i];
      // Handle right turns by symmetry
      if (curvature < 0) {
          point.y() *= -1;
      }

      // Check which one of the toruses the point lies within, if any
      float point_radius = sqrt(pow(point.x(), 2) + pow((radius - point.y()), 2)); // - note: no x-component because this is expressed in the car's base link
      float theta = atan2(point.x(), (radius - point.y()));

      // if point radius is < minimum radius of any point on car, it will never be an obstacle
      if (point_radius < inside_rear_axle_radius) {continue;}

      // likewise, if point radius is > than the maximum radius of any point on the car, it will never be an obstacle
      if (point_radius > std::max(outside_front_corner_radius, outside_rear_corner_radius)) {continue;}

      // Condition one: The point hits the inner side of the car
      // if radius is also less than the radius of the front inside corner
      if (((theta > 0)) && (point_radius >= inside_rear_axle_radius) && (point_radius < inside_front_corner_radius)) {
        float psi = acos(inside_rear_axle_radius / point_radius);
        float phi = theta - psi;
        // std::cout << "      A" << std::endl;
        if (radius * phi < free_path_length) {
          free_path_length = radius * phi;
        }
      }

      // Condition two: The point hits the front of the car
      // if radius also falls within the radii of the front corners
      else if ((theta > 0) && (inside_front_corner_radius <= point_radius) && (point_radius < outside_front_corner_radius)) {
        float psi = asin((margin_ + (car_->dimensions_.length_ + car_->dimensions_.wheelbase_) / 2) / point_radius);
        float phi = theta - psi;
        // std::cout << "      B" << std::endl;
        if (radius * phi < free_path_length) {
          free_path_length = radius * phi;
        }
      }

      // TODO Not needed for the assignment.
      // Condition three: The point hits the outer rear side of the car
      // if radius is greater than outside rear axle radius and less than the radius of the outside rear corner
      // if ((outside_rear_axle_radius <= point_radius) && (point_radius < outside_rear_corner_radius)) {
      //   if ((std::abs(point.x()) < margin_ + (car_->dimensions_.length_ - car_->dimensions_.wheelbase_) / 2) && (margin_ + (car_->dimensions_.width_ / 2) < std::abs(point.y()))) {
      //     float psi = -1 * acos(outside_rear_axle_radius / point_radius);
      //     float phi = theta - psi;
      //     if (radius * phi < free_path_length) {
      //       free_path_length = radius * phi;
      //     }
      //   }
      // }


    }
    //TODO
    //TODO
    //TODO For assignment 3, goal should be an input to this function. For now, just hard coding it here.
    //!!!!!!!!
    //. Limit free path length to closest point of approach
    Vector2f goal(10.0, 0);
    float theta = atan(goal.x()/radius);
    if (radius*theta < free_path_length){
      free_path_length = radius*theta;
    }
  }
  return free_path_length;
  // return std::max(free_path_length, 0.0f);
}

float Controller::calculateClearance(const std::vector<Vector2f>& point_cloud, const float curvature, const float free_path_length)
{
  Vector2f point(0.0, 0.0);
  float min_clearance = max_clearance_; // Note: We begin with a maximum clearance range of 0.5m - any obstacles further than this will not be checked. This should be tuned. 

  if (std::abs(curvature) < 0.01) { // Straight line case
    // Loop through point cloud
    for (int i = 0; i < (int)point_cloud.size(); i++) {
      point = point_cloud[i];
      // If the point lies between the car and the obstacle at the end of the free path, and within the side of the car and the maximum clearance, check clearance. If lower, replace.
      // TODO The second part of this could use a shortening (as described in class on 2/5/24)
      if ((car_->dimensions_.width_ / 2 + margin_ <= std::abs(point.y()) && std::abs(point.y()) <= max_clearance_) && (0 <= point.x() && (point.x() <= free_path_length + car_->dimensions_.wheelbase_))) {
        float clearance = std::abs(point.y()) - car_->dimensions_.wheelbase_ / 2 - margin_;
        if (clearance < min_clearance) {
          min_clearance = clearance;
        }
      }
    }
  } else { // Moving along an arc
    
    float radius {1.0f / curvature};
    // Handle right turn
    if (curvature < 0) {
      radius *= -1;
    }

    // Loop through point cloud
    for (int i = 0; i < (int)point_cloud.size(); i++) {
      point = point_cloud[i];
      if (curvature < 0) {
          point.y() *= -1;
      }

      float point_radius = sqrt(pow(point.x(), 2) + pow((radius - point.y()), 2));
      float theta = atan2(point.x(), (radius - point.y()));
      float phi = free_path_length / radius;

      // First check the points that lie along the free path
      if ((0 <= theta && theta <= phi) && (radius - car_->dimensions_.width_ / 2 - margin_ - max_clearance_ <= point_radius && point_radius <= radius + car_->dimensions_.width_ / 2 + margin_ + max_clearance_)) {
        float clearance = std::abs(point_radius * cos(theta) - radius) - car_->dimensions_.width_ / 2 - margin_;
        if (clearance < min_clearance) {
          min_clearance = clearance;
        }
      }

      // Then, check the points that will be next to the car at its final position
      Vector2f pos = utils::transforms::transformICOM(point.x(), point.y(), phi, radius);
      if ((car_->dimensions_.width_ / 2 + margin_ <= std::abs(pos.y()) && std::abs(pos.y()) <= max_clearance_) && (0 <= pos.x() && (pos.x() <=  car_->dimensions_.wheelbase_) / 2)) {
        float clearance = std::abs(pos.y()) - car_->dimensions_.width_ / 2 - margin_;
        if (clearance < min_clearance){
          min_clearance = clearance;
        }
      }
    }
  }
  return min_clearance;
}

// TODO
float Controller::calculateDistanceToGoal(const float curvature)
{
  Vector2f goal(10.0, 0.0);
  Vector2f projected_pos(0.0, 0.0);
  float goal_distance = 0;

  if (std::abs(curvature) < 0.01) {    // Straight line case
    projected_pos.x() = car_->limits_.max_speed_ * control_interval_;
    goal_distance = (goal-projected_pos).norm();
  }
  else {  // Moving along an arc
    float radius {1.0f / curvature};
    float phi = (car_->limits_.max_speed_ * control_interval_) / radius;
    projected_pos.x() = radius * sin(phi);
    projected_pos.y() = radius - (radius * cos(phi));
    goal_distance = (goal-projected_pos).norm();
  }

  return goal_distance;
}

PathCandidate Controller::evaluatePaths(const std::vector<Vector2f>& point_cloud)
{
  // creating starting path (with terrible score)
  auto best_path {PathCandidate(-100)};

  // weights
  float w1{3.0f}, w2{-0.5f};

  // Evaluate all possible paths and select optimal option
  for (float path_curvature = -1 * (car_->limits_.max_curvature_); path_curvature <= car_->limits_.max_curvature_; path_curvature += curvature_sampling_interval_) {
    
    // create candidate for this path
    PathCandidate candidate;
    candidate.curvature = path_curvature;

    // calculate free path length
    candidate.free_path_length = Controller::calculateFreePathLength(point_cloud, candidate.curvature);
    
    // calculate clearance
    candidate.clearance = Controller::calculateClearance(point_cloud, candidate.curvature, candidate.free_path_length);

    // ? do we include distance to goal now?
    candidate.goal_distance = Controller::calculateDistanceToGoal(candidate.curvature);

    // Calculate score and update selection
    candidate.score = candidate.free_path_length + w1 * candidate.clearance + w2 * candidate.goal_distance;
    
    if (candidate.score > best_path.score) {
      best_path = candidate;
    }
  }

  // return best path
  return best_path;
}

ControlCommand Controller::generateCommand(const std::vector<Vector2f>& point_cloud, const float current_speed)
{
  PathCandidate path {Controller::evaluatePaths(point_cloud)};

  std::cout << path.curvature << " " << path.free_path_length << " " << path.clearance << " " << path.goal_distance << " " << path.score << std::endl;

  float speed {Controller::calculateControlSpeed(current_speed, path.free_path_length)};
  return ControlCommand(speed, path.curvature);
}

} // namespace time_optimal_1D

// -------------------------------------------------------------------------
// * LATENCY COMPENSATION
// -------------------------------------------------------------------------

namespace latency_compensation {

// -------------------------------------------------------------------------
// & constructor & destructor
// -------------------------------------------------------------------------
Controller::Controller(vehicles::Car *car, float control_interval, float margin, float max_clearance, float curvature_sampling_interval, float latency) : car_(car), latency_(latency)
{
  // create a new TimeOptimalController to use
  toc_ = new time_optimal_1D::Controller(car, control_interval, margin, max_clearance, curvature_sampling_interval);
}

Controller::~Controller()
{
  delete toc_;
}

// -------------------------------------------------------------------------
// & adding to command history
// -------------------------------------------------------------------------
void Controller::recordCommand(const LatentCommand command)
{
  // add the command to the command history
  command_history_.push(command);
}

void Controller::recordCommand(const float v, const float c)
{
  // add the command to the command history
  Controller::recordCommand(LatentCommand(v, c));
}

// -------------------------------------------------------------------------
// & projecting forward
// -------------------------------------------------------------------------
// TODO project the car forward however many timesteps we want
// TODO transform points into that frame
// TODO pass the updated information to the 1D time-optimal controller

// void LatencyController::projectForward(std::chrono::milliseconds timestamp)
// {
  
// }

// void LatencyController::updateController(std::vector<Eigen::Vector2f> point_cloud)
// {
  // using latency, and history of results, project the car's position and velocity forward through time
  // LatencyController::projectForward();

  // use this forward projection to transform the point cloud

  // feed these updated parameters into the 1D time-optimal controller

  // receive a response from the 1D TOC and record it, then bubble it back out to the main

// }

} // namespace latency_compensation

} // namespace controllers

// -------------------------------------------------------------------------
// * FIN
// -------------------------------------------------------------------------
