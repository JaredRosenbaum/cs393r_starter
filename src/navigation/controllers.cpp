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
: car_(car), control_interval_(control_interval), margin_(margin), max_clearance_(max_clearance), curvature_sampling_interval_(curvature_sampling_interval), previous_curvature_(0.f)
{}

float Controller::calculateControlSpeed(float current_speed, const float free_path_length)
{
  float control_speed {0.0};
  
//  current_speed = int(current_speed*5)/5.f;
  if ((current_speed >= car_->limits_.max_speed_ - 0.05) && (current_speed <= car_->limits_.max_speed_ + 0.05)){
	current_speed = car_->limits_.max_speed_;
  }

  // Case 1: Accelerate
  if ((current_speed < car_->limits_.max_speed_) && 
      (free_path_length >= current_speed * control_interval_ + (car_->limits_.max_acceleration_ * control_interval_) * control_interval_ / 2 + pow((current_speed + car_->limits_.max_acceleration_ * control_interval_), 2) / (2 * car_->limits_.max_acceleration_))) {
        control_speed = current_speed + car_->limits_.max_acceleration_ * control_interval_;  // speed increases by acceleration rate
  }
  // Case 2: Cruise
  // Note: For now, we are taking a small volume around car_->limits_.max_speed_. Might need to move this to navigation.cc
  else if ((current_speed == car_->limits_.max_speed_) && (free_path_length >= current_speed * control_interval_ + car_->limits_.max_speed_ * car_->limits_.max_speed_ / (2 * car_->limits_.max_acceleration_))) {
        control_speed = current_speed;
  }
  // Case 3: Decelerate
  else if (free_path_length < pow((current_speed), 2) / (2 * car_->limits_.max_acceleration_)) {
        control_speed = current_speed - car_->limits_.max_acceleration_ * control_interval_;  // speed decreases by acceleration rate
  }
  // Case 4: Declerate with expected collision warning
  else {
        control_speed = current_speed - car_->limits_.max_acceleration_ * control_interval_;  // speed decreases by acceleration rate
        // std::cout << "Not enough room to decelerate! Expecting collision..." << std::endl;
        // std::cout << "The free path length is: " << free_path_length << std::endl;
  }

  if (control_speed < 0) { // prevent reversal
    control_speed = 0;
  }
  if (control_speed > car_->limits_.max_speed_) {
    control_speed = car_->limits_.max_speed_;
  }
  
  return control_speed;
}

// float Controller::calculateDistanceToGoal(const float curvature)
// {
//   // Vector2f goal(10.0, 0.0);
//   Vector2f projected_pos(0.0, 0.0);
//   float goal_distance = 0;

//   if (std::abs(curvature) < 0.01) {    // Straight line case
//     projected_pos.x() = car_->limits_.max_speed_ * control_interval_;
//     goal_distance = (goal-projected_pos).norm();
//   }
//   else {  // Moving along an arc
//     float radius {1.0f / curvature};
//     float phi = (car_->limits_.max_speed_ * control_interval_) / radius;
//     projected_pos.x() = radius * sin(phi);
//     projected_pos.y() = radius - (radius * cos(phi));
//     goal_distance = (goal-projected_pos).norm();
//   }
//   return goal_distance;
// }

Command Controller::generateCommand(const std::vector<Vector2f>& point_cloud, const float current_speed, std::vector<path_generation::Path> &paths, path_generation::Path &best_path, const Vector2f goal, const Vector2f global_goal)
{
  paths = path_generation::samplePathOptions(N_PATHS, point_cloud, *car_, goal, global_goal);
  int best_path_index {path_generation::selectPath(paths)};
  best_path = paths[best_path_index];
  

  float speed {Controller::calculateControlSpeed(current_speed, paths[best_path_index].free_path_length)};

  return Command(speed, paths[best_path_index].curvature);
}

float Controller::getControlInterval()
{
  return control_interval_;
}

} // namespace time_optimal_1D

// -------------------------------------------------------------------------
// * LATENCY COMPENSATION
// -------------------------------------------------------------------------

namespace latency_compensation {

// -------------------------------------------------------------------------
// & constructor & destructor
// -------------------------------------------------------------------------
// Controller::Controller(vehicles::Car car, float control_interval, float margin, float max_clearance, float curvature_sampling_interval, float latency) : car_(car), latency_(latency)
Controller::Controller(vehicles::Car *car, float control_interval, float margin, float max_clearance, float curvature_sampling_interval, float latency) : latency_(latency)
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
void Controller::recordCommand(const CommandStamped command)
{
  // add the command to the command history
  command_history_.push_back(command);
  // std::cout << "New command recorded for timestamp " << command.timestamp << std::endl;
}

void Controller::recordCommand(const time_optimal_1D::Command command)
{
  // add the command to the command history
  Controller::recordCommand(CommandStamped(command));
}

// -------------------------------------------------------------------------
// & projecting forward
// -------------------------------------------------------------------------

time_optimal_1D::Command Controller::generateCommand(const std::vector<Vector2f>& point_cloud, const float current_speed, const double last_data_timestamp, std::vector<path_generation::Path> &paths, path_generation::Path &best_path, const Vector2f goal, const Vector2f global_goal)
{
  // using latency, and history of results, project the car's position and velocity forward through time; search the controls queue and pop until a timestamp is newer than the current time
  State2D projected_state {Controller::projectState(current_speed, last_data_timestamp)};

  // use this forward projection to transform the point cloud
  auto cloud {Controller::transformCloud(point_cloud, projected_state)};

  // feed these updated parameters into the 1D time-optimal controller
  time_optimal_1D::Command command {toc_->generateCommand(cloud, projected_state.speed, paths, best_path, goal, global_goal)};

  // receive a response from the 1D TOC and record it, then bubble it back out to the main
  Controller::recordCommand(command);

  return command;
}

State2D Controller::projectState(const float current_speed, const double last_msg_timestamp)
{
  // setting state to reflect the starting state of the robot
  State2D state;
  state.speed = current_speed;
  state.position = Eigen::Vector2f {0.f, 0.f};
  state.theta = 0;

  if (command_history_.size() < 1) {
    return state;
  }

  double time_threshold {ros::Time::now().toSec()};
  while (!command_history_.empty()) {
    if ((time_threshold - command_history_.front().timestamp) < latency_) {
      break;
    }
//    std::cout << "Removing command with diff " << time_threshold - command_history_.front().timestamp << " from command history for latency " << latency_ << std::endl;
    command_history_.pop_front();
  }

  // project the future state of the car
//  std::cout << "Considering " << command_history_.size() << " previous commands to compensate for latency..." << std::endl;
  for (const auto &command : command_history_) {
    // std::cout << "Latency: " << latency_ << ", Diff: " << command.timestamp - last_msg_timestamp << std::endl;
    double distance_traveled {command.command.velocity * toc_->getControlInterval()};

    // std::cout << "Speed: " << command.command.velocity << ", Curvature: " << command.command.curvature << std::endl;
    if (std::abs(command.command.curvature) > 0.01) { // updating for curved case
      double radius {1 / command.command.curvature};
      double theta {distance_traveled / radius};
      state.position.x() += distance_traveled * cos(theta);
      state.position.y() += distance_traveled * sin(theta);
      state.theta += theta;
    } else { // updating for straight case
      state.position.x() += distance_traveled;
    }
    state.speed = command.command.velocity;

    // std::cout << "Updated state: ";
    // Controller::printState(state);
  }

  // std::cout << "Returning state: ";
  // Controller::printState(state);
  return state;
}

void Controller::printState(const State2D &state)
{
  std::cout << "State is: \n\tPosition:\t(" << state.position.x() << ", " << state.position.y() << ")\n\tTheta:\t\t" << state.theta << "\n\tSpeed:\t\t" << state.speed << std::endl;
}

std::vector<Eigen::Vector2f> Controller::transformCloud(std::vector<Eigen::Vector2f> cloud, const State2D &state)
{
  // generate a transformation matrix from projected state
  Eigen::Matrix3f transformation_matrix;
  transformation_matrix << 
        cos(state.theta), -1 * sin(state.theta), state.position.x(),
        sin(state.theta),      cos(state.theta), state.position.y(),
                       0,                     0,                  1;

  // TODO expressly taking the inverse is more expensive than it needs to be, consider replacing this with a more efficient split-up calculation
  auto inv_transformation_matrix {transformation_matrix.inverse()};

  // use it to transform the cloud points
  for (std::size_t i = 0; i < cloud.size(); i++) {
    // std::cout << "Point before: " << cloud[i].transpose() << std::endl;
    Eigen::Vector3f augmented_point {cloud[i].x(), cloud[i].y(), 1};
    Eigen::Vector3f transformed_point {inv_transformation_matrix * augmented_point};
    cloud[i] = Eigen::Vector2f(transformed_point.x(), transformed_point.y());
    // std::cout << "Point after: " << cloud[i].transpose() << std::endl;
  }
  return cloud;
}

} // namespace latency_compensation

} // namespace controllers

// -------------------------------------------------------------------------
// * FIN
// -------------------------------------------------------------------------
