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
: car_(car), control_interval_(control_interval), margin_(margin), max_clearance_(max_clearance), curvature_sampling_interval_(curvature_sampling_interval) 
{}

float Controller::calculateControlSpeed(float current_speed, const float free_path_length)
{
  float control_speed {0.0};
  float v_max = car_->limits_.max_speed_; //Maximum velocity of the car
  float a_max = car_->limits_.max_acceleration_; //Maximum acceleration of the car
  float dt = control_interval_; //Control interval
  
  current_speed = int(current_speed*5)/5.f;


  //. Case 1: Accelerate
  if ((current_speed < car_->limits_.max_speed_) && 
      (free_path_length >= current_speed*dt + (a_max*dt)*dt/2 + pow((current_speed+a_max*dt),2)/(2*a_max))) {
        control_speed = current_speed + car_->limits_.max_acceleration_ * control_interval_;  // speed increases by acceleration rate
  }  
  // Note: For now, we are taking a small volume around v_max. Might need to move this to navigation.cc
  else if ((current_speed == v_max) && (free_path_length >= current_speed*dt + v_max*v_max/(2*a_max))) {
        control_speed = current_speed;
  }
  else if (free_path_length < pow((current_speed),2)/(2*a_max)){
        control_speed = current_speed - car_->limits_.max_acceleration_ * control_interval_;  // speed decreases by acceleration rate
  }
  else {
        control_speed = current_speed - car_->limits_.max_acceleration_ * control_interval_;  // speed decreases by acceleration rate
        std::cout << "I made it into the weird, fourth case!" << std::endl;
        std::cout << "The free path length is: " << free_path_length << std::endl;
        std::cout << "The area under the triangle is: " << pow((current_speed),2)/(2*a_max) << std::endl;
  }

if (control_speed < 0) { // prevent reversal
  control_speed = 0;
}
if (control_speed > v_max) {
  control_speed = 1;
}
  // Accelerate Case
  // Use kinematic equations to check if we have enough distance to accelerate one more time-step
  // if (current_speed < car_->limits_.max_speed_ && free_path_length > ((current_speed * control_interval_) + ((current_speed + car_->limits_.max_acceleration_ * pow(control_interval_, 2)) / 2 ) + (pow(current_speed + car_->limits_.max_acceleration_ * control_interval_, 2) / 2 * car_->limits_.max_acceleration_))) {
  //   control_speed = current_speed + car_->limits_.max_acceleration_ * control_interval_;  // speed increases by acceleration rate
  //   control_speed = car_->limits_.max_speed_;
  // }
  // // Cruise Case
  // // Use kinematic equations to check if we have enough distance to cruise one more time-step
  // else if (current_speed == car_->limits_.max_speed_ && free_path_length > 
  // ((current_speed * control_interval_) + (pow(current_speed, 2) / 2 * car_->limits_.max_acceleration_))) {
  //   control_speed = current_speed; // speed remains the same
  // }
  // // Decelerate Case
  // // Not enough distance, calculate deceleration rate
  // else {
  //   // TODO Remove since we are using a slam on the brakes approach
  //   // float d = pow(current_speed, 2) / (2 * free_path_length);
  //   // if ((d < 0) || (std::abs(d) > std::abs(car_->limits_.max_acceleration_))) {
  //   //   d = car_->limits_.max_acceleration_;
  //   // }
  //   // control_speed = current_speed - d * control_interval_;  // speed decreases by acceleration rate
  //   control_speed = current_speed - car_->limits_.max_acceleration_ * control_interval_;  // speed decreases by acceleration rate
  //   if (control_speed < 0)  // prevent reversal
  //     control_speed = 0;
  //   control_speed = 0;
  // }

  return control_speed;
}

// float Controller::calculateFreePathLength(const std::vector<Vector2f>& point_cloud, const float curvature)
// {
//   // line case
//   if (abs(curvature) < 0.01) {
//     // TODO handle straight case
//     return 0.0f;
//   }

//   // arc case

//   // create vector for turning radius
//   float turning_radius_magnitude {1.0f / curvature};
//   Eigen::Vector2f turning_radius {0.0f, turning_radius_magnitude};

//   // calculate other useful vectors
//   Eigen::Vector2f inside_rear_axle {0.0f, margin_ + car_->dimensions_.width_ / 2};
//   Eigen::Vector2f inside_front_corner {margin_ + (car_->dimensions_.length_ + car_->dimensions_.wheelbase_) / 2, margin_ + car_->dimensions_.width_ / 2};
//   Eigen::Vector2f outside_front_corner {inside_front_corner.x(), -1 * inside_front_corner.y()};
//   Eigen::Vector2f outside_rear_corner {margin_ + (car_->dimensions_.length_ - car_->dimensions_.wheelbase_) / 2, margin_ + car_->dimensions_.width_ / 2};
//   Eigen::Vector2f outside_rear_axle {0.0f, -1 * inside_rear_axle.y()};

//   // flipping signs on y-components (because inside/outside is relative to the direction you're turning)
//   if (curvature < 0) {
//     inside_rear_axle.y() = -1 * inside_rear_axle.y();
//     inside_front_corner.y() = -1 * inside_front_corner.y();
//     outside_front_corner.y() = -1 * outside_front_corner.y();
//     outside_rear_corner.y() = -1 * outside_rear_corner.y();
//     outside_rear_axle.y() = -1 * outside_rear_axle.y();
//   }

//   // sqrd magntiude calculations (so they don't need to be repeated)
//   double inside_rear_axle_sqrd {pow(inside_rear_axle.x(), 2) + pow(inside_rear_axle.y(), 2)}; 
//   double inside_front_corner_sqrd {pow(inside_front_corner.x(), 2) + pow(inside_front_corner.y(), 2)};
//   double outside_front_corner_sqrd {pow(outside_front_corner.x(), 2) + pow(outside_front_corner.y(), 2)};
//   double outside_rear_corner_sqrd {pow(outside_rear_corner.x(), 2) + pow(outside_rear_corner.y(), 2)};
//   double outside_rear_axle_sqrd {pow(outside_rear_axle.x(), 2) + pow(outside_rear_axle.y(), 2)};

//   // loop over points
//   for (int i = 0; i < (int)point_cloud.size(); i++) {

//     // . calculating vector and magnitude
//     Eigen::Vector2f point {point_cloud[i]};
//     double point_sqrd {pow(point.x(), 2) + pow(point.y(), 2)};
//     // double point_magnitude {sqrt(point_sqrd)};

//     // . calculating angle between point and radius vectors
//     double theta {atan2(point.y() - turning_radius.y(), point.x() - turning_radius.x())};

//     std::cout << "\tPoint: (" << point.x() << ", " << point.y() << "), theta: " << theta << std::endl;

//     // . broad-face checks
//     // radius too small to ever be an obstacle
//     if (point_sqrd < inside_rear_axle_sqrd) {
//       std::cout << "\t\tToo small!" << std::endl;
//       continue;
//     }
//     // radius too large to ever be an obstacle
//     if (std::max(outside_front_corner_sqrd, outside_rear_corner_sqrd) < point_sqrd) {
//       std::cout << "\t\tToo large!" << std::endl;
//       continue;
//     }

//     // . narrow-face checks
//     // point hits inside of car
//     if ((inside_rear_axle_sqrd < point_sqrd) && (point_sqrd < inside_front_corner_sqrd)) {
//       // TODO
//       std::cout << "\t\tSide obstacle!" << std::endl;
//       // double psi {atan2(inside_rear_axle.y() - turning_radius.y(), inside_rear_axle.x() - turning_radius.x())};
//     }

//     // point hits front of car
//     if ((inside_front_corner_sqrd <= point_sqrd) && (point_sqrd <= outside_front_corner_sqrd)) {
//       // TODO
//       // ? how can I get the orientation of the vector for the front collision? I know it's length, but don't know its orientation -> if I get this, I can compute the atan(x_diff, y_diff) to get the angle I care about
//       std::cout << "\t\tFront obstacle!" << std::endl;
//       // double psi {asin((margin_ + (car_->dimensions_.length_ + car_->dimensions_.wheelbase_) / 2) / point_magnitude)};
//     }

//     // point hits rear back corner of car
//     if ((outside_rear_axle_sqrd < point_sqrd) && (point_sqrd < outside_rear_corner_sqrd)) {
//       // TODO
//       std::cout << "\t\tRear obstacle!" << std::endl;
//     }
//   }
//   return 0.0f;
// }

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
  float w1{5.0f}, w2{-0.5f};

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

Command Controller::generateCommand(const std::vector<Vector2f>& point_cloud, const float current_speed)
{
  PathCandidate path {Controller::evaluatePaths(point_cloud)};

  //std::cout << path.curvature << " " << path.free_path_length << " " << path.clearance << " " << path.goal_distance << " " << path.score << std::endl;

  float speed {Controller::calculateControlSpeed(current_speed, path.free_path_length)};
  std::cout << "FPL: " << path.free_path_length << ", " << "Current speed: " << current_speed << ", " << "Commanded speed: " << speed << std::endl;
  return Command(speed, path.curvature);
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

time_optimal_1D::Command Controller::generateCommand(const std::vector<Vector2f>& point_cloud, const float current_speed, const double last_data_timestamp)
{
  // using latency, and history of results, project the car's position and velocity forward through time; search the controls queue and pop until a timestamp is newer than the current time
  State2D projected_state {Controller::projectState(current_speed, last_data_timestamp)};

  // use this forward projection to transform the point cloud
  auto cloud {Controller::transformCloud(point_cloud, projected_state)};

  // feed these updated parameters into the 1D time-optimal controller
  time_optimal_1D::Command command {toc_->generateCommand(cloud, projected_state.speed)};

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
    // ! below was compensating for sensing latency, what we really care about is actuation latency
    // if ((command_history_.front().timestamp + latency_) >= last_msg_timestamp) {
    //   break;
    // }
    // std::cout << "Removing command with stamp difference " << (command_history_.front().timestamp + latency_) - last_msg_timestamp << " from history." << std::endl;
    // + this should be handling actuation latency
    if ((time_threshold - command_history_.front().timestamp) < latency_) {
      break;
    }
    std::cout << "Removing command with diff " << time_threshold - command_history_.front().timestamp << " from command history for latency " << latency_ << std::endl;
    command_history_.pop_front();
  }

  // project the future state of the car
  std::cout << "Considering " << command_history_.size() << " previous commands to compensate for latency..." << std::endl;
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

float Controller::calculateFreePathLength(const std::vector<Vector2f>& point_cloud, const float curvature, const double last_data_timestamp)
{
  // using latency, and history of results, project the car's position and velocity forward through time; search the controls queue and pop until a timestamp is newer than the current time
  State2D projected_state {Controller::projectState(0.f, last_data_timestamp)};

  // use this forward projection to transform the point cloud
  auto cloud {Controller::transformCloud(point_cloud, projected_state)};

  // feed these updated parameters into the 1D time-optimal controller
  float fpl {toc_->calculateFreePathLength(cloud, curvature)};
  return fpl;
}



} // namespace latency_compensation

} // namespace controllers

// -------------------------------------------------------------------------
// * FIN
// -------------------------------------------------------------------------
