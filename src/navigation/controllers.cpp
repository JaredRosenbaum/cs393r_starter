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

float Controller::calculateFreePathLength(const std::vector<Vector2f>& point_cloud, float curvature)
{
  float free_path_length {10.0f - (margin_ + (car_->dimensions_.length_ + car_->dimensions_.wheelbase_) / 2)}; // TODO set this properly
  float candidate_free_path_length {0.f};
  Vector2f point(0.0, 0.0);

  // !!!!!!!!!! BORKEN
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
    float outside_rear_axle_radius {radius + (margin_ + car_->dimensions_.width_ / 2)};

    // Loop through point cloud
    for (int i = 0; i < (int)point_cloud.size(); i++) {
      point = point_cloud[i];
      // Handle right turns by symmetry
      if (curvature < 0) {
          point.y() *= -1;
      }

      // Check which one of the toruses the point lies within, if any
      float point_radius = sqrt(pow(point.x(), 2) + pow((radius - point.y()), 2));
      float theta = atan2(point.x(), (radius - point.y()));

      // if point radius is < minimum radius of any point on car, it will never be an obstacle
      if (point_radius < inside_rear_axle_radius) {continue;}

      // likewise, if point radius is > than the maximum radius of any point on the car, it will never be an obstacle
      if (point_radius > std::max(outside_front_corner_radius, outside_rear_corner_radius)) {continue;}

      // . Condition one: The point hits the inner side of the car
      // if radius is also less than the radius of the front inside corner
      if ((point_radius >= inside_rear_axle_radius) && (point_radius < inside_front_corner_radius) && ((theta > 0))) {
        float psi = acos(inside_rear_axle_radius / point_radius);
        float phi = theta - psi;
        // std::cout << "      A" << std::endl;
        if (radius * phi < free_path_length) {
          free_path_length = radius * phi;
        }
      }

      // . Condition two: The point hits the front of the car
      // if radius also falls within the radii of the front corners
      else if ((inside_front_corner_radius <= point_radius) && (point_radius < outside_front_corner_radius) && (theta > 0)) {
        float psi = asin((margin_ + (car_->dimensions_.length_ + car_->dimensions_.wheelbase_) / 2) / point_radius);
        float phi = theta - psi;
        // std::cout << "      B" << std::endl;
        if (radius * phi < free_path_length) {
          free_path_length = radius * phi;
        }
      }

      // . Condition three: The point hits the outer rear side of the car
      // if radius is greater than outside rear axle radius and less than the radius of the outside rear corner
      if ((outside_rear_axle_radius <= point_radius) && (point_radius < outside_rear_corner_radius)) {
        if ((std::abs(point.x()) < margin_ + (car_->dimensions_.length_ - car_->dimensions_.wheelbase_) / 2) && (margin_ + (car_->dimensions_.width_ / 2) < std::abs(point.y()))) {
          float psi = -1 * acos(outside_rear_axle_radius / point_radius);
          float phi = theta - psi;
          if (radius * phi < free_path_length) {
            candidate_free_path_length = radius * phi;
            if (candidate_free_path_length) {}
          }
        }
      }

    }
    //TODO Limit free path length to closest point of approach
    // Vector2f goal(10.0, 0);
    // float theta = atan(goal.x()/radius);
    // // float theta = atan2(radius, goal.x());
    // // float theta = atan2(goal.x(), radius);
    // if (radius*theta < free_path_length){
    //   free_path_length = radius*theta;
    // }
  }
  return free_path_length;
}

// float Controller::calculateClearance(const std::vector<Vector2f>& point_cloud, const float curvature, const float free_path_length)
void Controller::calculateClearance(const std::vector<Vector2f>& point_cloud, PathCandidate &path) 
{
  Vector2f point(0.0, 0.0);
  float min_clearance = max_clearance_; // Note: We begin with a maximum clearance range of 0.5m - any obstacles further than this will not be checked. This should be tuned. 

  if (std::abs(path.curvature) < 0.01) { // Straight line case
    // Loop through point cloud
    for (int i = 0; i < (int)point_cloud.size(); i++) {
      point = point_cloud[i];
      // If the point lies between the car and the obstacle at the end of the free path, and within the side of the car and the maximum clearance, check clearance. If lower, replace.
      // TODO The second part of this could use a shortening (as described in class on 2/5/24)
      if ((car_->dimensions_.width_ / 2 + margin_ <= std::abs(point.y()) && std::abs(point.y()) <= max_clearance_) && (0 <= point.x() && (point.x() <= path.free_path_length + car_->dimensions_.wheelbase_))) {
        float clearance = std::abs(point.y()) - car_->dimensions_.wheelbase_ / 2 - margin_;
        if (clearance < min_clearance) {
          min_clearance = clearance;
        }
      }
    }
  } else { // Moving along an arc
    
    float radius {1.0f / path.curvature};
    // Handle right turn
    if (path.curvature < 0) {
      radius *= -1;
    }

    // Loop through point cloud
    for (int i = 0; i < (int)point_cloud.size(); i++) {
      point = point_cloud[i];
      if (path.curvature < 0) {
          point.y() *= -1;
      }

      float point_radius = sqrt(pow(point.x(), 2) + pow((radius - point.y()), 2));
      float theta = atan2(point.x(), (radius - point.y()));
      float phi = path.free_path_length / radius;
      // First check the points that lie along the free path
      if ((0 <= theta && theta <= phi) && (radius - car_->dimensions_.width_ / 2 - margin_ - max_clearance_ <= point_radius && point_radius <= radius + car_->dimensions_.width_ / 2 + margin_ + max_clearance_)) {
        float clearance = std::abs(point_radius * cos(theta) - radius) - car_->dimensions_.width_ / 2 - margin_;
        if (clearance < min_clearance) {
          min_clearance = clearance;
        }
      }
      // // Then, check the points that will be next to the car at its final position
      // Vector2f pos = utils::transforms::transformICOM(point.x(), point.y(), phi, radius);
      // if ((car_->dimensions_.width_ / 2 + margin_ <= std::abs(pos.y()) && std::abs(pos.y()) <= max_clearance_) && (0 <= pos.x() && (pos.x() <=  car_->dimensions_.wheelbase_) / 2)) {
      //   float clearance = std::abs(pos.y()) - car_->dimensions_.width_ / 2 - margin_;
      //   if (clearance < min_clearance){
      //     min_clearance = clearance;
      //   }
      // }
    }
  }
  path.clearance = min_clearance;
}


// void Controller::calculateClearance(const std::vector<Vector2f>& point_cloud, PathCandidate &path) 
// {
//   static const bool kDebug = false;
//   // static const bool kDebug = true;

//   path.clearance = MAX_CLEARANCE;

//   const float l = car_->dimensions_.length_ + 2 * margin_;
//   const float w = car_->dimensions_.length_ + 2 * margin_;
//   const float l_f = l - (l - car_->dimensions_.wheelbase_) / 2;  // base to front
//   const float l_r = l - l_f;                                    // base to rear

//   // std::cout << l << "\t" << w << "\t" << l_f << "\t" << l_r << std::endl;

//   // Add special case to handle when car is driving nearly straight
//   if (std::abs(path.curvature) < 1e-5) {
//     // Only check if points are within the width of the car
//     const float min_y = -w / 2.0f;
//     const float max_y = w / 2.0f;

//     for (const auto &point : point_cloud) {
//       // Point is outside the lateral swept volume of the car
//       if (point.y() < min_y || point.y() > max_y) {
//         const float clearance = std::min(std::abs(point.y() - min_y), std::abs(point.y() - max_y));
//         // path_ptr->set_clearance(std::min(clearance, path.clearance()));
//         path.clearance = std::min(clearance, path.clearance);
//         continue;
//       }

//       // Point is outside the forward swept volume of the car
//       if (point.x() > path.free_path_length + l_f || point.x() < -l_r) {
//         continue;
//       }

//       // Point in inside the swept volume of the car
//       float arc_length = std::max(point.x() - l_f, 0.f);
//       // path_ptr->set_arc_length(fmin(arc_length, path_ptr->arc_length()));
//       // path_ptr->set_clearance(0);
//       path.free_path_length = std::min(arc_length, path.free_path_length);
//       path.clearance = 0;
//       break;
//     }
//     return;
//   }

//   // compute volume swept by the car during a single time (depends on curvature)
//   const float r = 1 / path.curvature;
//   const float r_base_min = std::abs(r) - (w / 2.0f);
//   const float r_base_max = std::abs(r) + (w / 2.0f);
//   const float r_front_min = sqrt(Sq(r_base_min) + Sq(l_f));
//   const float r_front_max = sqrt(Sq(r_base_max) + Sq(l_f));
//   const float r_rear_max = sqrt(Sq(r_base_max) + Sq(l_r));

//   const Eigen::Vector2f instant_center(0, r);
//   for (const auto &point : point_cloud) {

//     if (point.x() < (-1 * l_r)) {
//       // Case 1: Point is behind the car
//       continue;
//     }

//     const float r_p = (point - instant_center).norm();
//     // std::cout << r_p << "\t" << r_base_min << "\t" << r_front_max << std::endl;
//     if (r_p < r_base_min || r_p > r_front_max) {
//       // Case 2: Point is out of sweep volume
//       const float clearance = std::min(std::abs(r_p - r_base_min), std::abs(r_p - r_front_max));
//       // path_ptr->set_clearance(fmin(clearance, path_ptr->clearance()));
//       path.clearance = std::min(clearance, path.clearance);
//       continue;
//     }

//     int phase = 0;
//     Eigen::Vector2f point_to_collision(0, 0);
//     if (r_p < r_front_min) {
//       // Case 3: Point is in swept volume in inside of car
//       point_to_collision.y() = Sign(r) * (w / 2.0f);
//       point_to_collision.x() = sqrt(Sq(r_p) - Sq(point_to_collision.y() - r));
//       if (kDebug) {
//         std::cout << point.transpose() << " is inner area of sweep volume" << std::endl;
//         phase = 3;
//       }
//     } else if (r_p > r_base_max && r_p < r_rear_max && fabs(point.x()) < l_r) {
//       // Case 4: Point is in sweapt volum  between base_link and rear bumper of car
//       point_to_collision.y() = -Sign(r) * (w / 2.0f);
//       point_to_collision.x() = sqrt(Sq(r_p) - Sq(point_to_collision.y() - r));
//       if (kDebug) {
//         std::cout << point.transpose() << " is in rear area of sweep volume" << std::endl;
//         phase = 4;
//       }
//     } else if (r_p < r_front_max && point.x() > l_f) {
//       // Case 5: Point is in swept volume in front of car
//       point_to_collision.x() = l_f;
//       point_to_collision.y() = r - Sign(r) * sqrt(Sq(r_p) - Sq(point_to_collision.x()));
//       if (kDebug) {
//         std::cout << point.transpose() << " is in front area of sweep volume" << std::endl;
//         phase = 5;
//       }
//     } else {
//       // Case 6: Point is out of swept volume
//       const float clearance = std::abs(r_rear_max - r_p);
//       path.clearance = std::min(clearance, path.clearance);
//       if (kDebug) {
//         std::cout << point.transpose() << " is out of sweep volume" << std::endl;
//         phase = 6;
//       }
//       continue;
//     }

//     // std::cout << "\tPhase: " << phase << std::endl;

//     // Compute the arc length until the collision
//     const Eigen::Vector2f collision_radial = point_to_collision - instant_center;
//     const Eigen::Vector2f point_radial = point - instant_center;
//     const float swap_theta = acos(point_radial.dot(collision_radial) /
//                                   (point_radial.norm() * collision_radial.norm()));
//     const float arc_length = std::abs(r) * swap_theta;

//     if (kDebug) {
//       std::cout << "Curvature: " << path.curvature
//            << ", Arc length to collision: " << arc_length << std::endl;
//       std::cout << "Point to collision: " << point_to_collision.transpose()
//            << ", r_p: " << r_p << ", r: " << r << ", swap_theta: " << swap_theta
//            << ", phase: " << phase << std::endl;
//       std::cout << phase << std::endl;
//     }

//     // Penalize clearance as well if we have a collision before arc length is traversed
//     // if (arc_length < path.free_path_length) {
//     //   path.free_path_length = arc_length;
//     //   path.clearance = 0;
//     // }
//   }
// }


// set curvature, free path length, obstruction for a path option
void Controller::computePathOption(PathCandidate &path_option, float curvature, const std::vector<Eigen::Vector2f>& point_cloud) {
    path_option.curvature = curvature;
    float h = (car_->dimensions_.length_ + car_->dimensions_.wheelbase_) / 2; // distance from base link to front bumper
    
    if (curvature < 1e-5) {
        for (const auto &p: point_cloud) {
            if (car_->dimensions_.width_/2 + CAR_MARGIN >= abs(p[1])
                && p[0] < path_option.free_path_length) {
                path_option.free_path_length = p[0] - h - CAR_MARGIN;
                // path_option.obstruction = p;
            }
        }
        // clearance
        for (const auto &p: point_cloud) {
            if (p[0] >=0 and p[0] < path_option.free_path_length) {
                float clearance_p = abs(p[1]) - car_->dimensions_.width_ / 2 - CAR_MARGIN;
                if (clearance_p < path_option.clearance) {
                    path_option.clearance = clearance_p;
                    // path_option.closest_point = p;
                }
            }
        }
        return;
    }

    Vector2f c = Vector2f(0, 1 / curvature);
    float r_inner = c.norm() - car_->dimensions_.width_ / 2 - CAR_MARGIN;
    float r_outer = c.norm() + car_->dimensions_.width_ / 2 + CAR_MARGIN;
    float r_tl = (Vector2f(0, r_inner) - Vector2f(h + CAR_MARGIN, 0)).norm();
    float r_tr = (Vector2f(0, r_outer) - Vector2f(h + CAR_MARGIN, 0)).norm();
    float r_br = (Vector2f(0, r_outer) - Vector2f((car_->dimensions_.length_ - car_->dimensions_.wheelbase_) / 2 + CAR_MARGIN, 0)).norm();

    path_option.free_path_length = std::min(M_PI * c.norm(), 5.0);  // some large number
    // float omega = atan2(h, r_inner);

    float theta_br = asin((car_->dimensions_.length_ - car_->dimensions_.wheelbase_) / 2 + CAR_MARGIN / r_br); // angle where back right would hit

    float phi = 0;
//	cout << "curvature " << curvature << endl;
//	bool front_side = false, outer_side = false, inner_side = false;
    for (unsigned int i = 0; i < point_cloud.size(); i++) {
        Vector2f p = point_cloud[i];
        float r_p = (c-p).norm();
        float theta = curvature < 0 ? atan2(p[0], p[1]- c[1]) : atan2(p[0], c[1] - p[1]); // angle between p and c
        float length = 5.0;
        // cout << "curvature " << curvature << endl;
        if (r_inner <= r_p && r_p <= r_tl) {    // inner side hit
                phi = acos(r_inner / r_p);
                length = (theta - phi) * c.norm();
            // inner_side = true;
                // cout << "inner side hit" << endl;
        }
        if ((r_inner <= r_p && r_p <= r_br) && (-theta_br <= theta && theta <= theta_br)) {    // outer side hit
            phi = acos(r_p / (c.norm() + car_->dimensions_.width_ / 2));

            length = (theta - phi) * c.norm();
	    // outer_side = true;
	    // cout << "outer side hit" << endl;
        }

        if (r_tl <= r_p && r_p <= r_tr) {    // front side hit
            phi = asin(h / r_p);
            length = (theta - phi) * c.norm();
	    // front_side = true;
	    // cout << "front side hit" << endl;
        }
        if (length < path_option.free_path_length && length > 0) {
            path_option.free_path_length = length;
            // path_option.obstruction = p;
        }
    }
	// if (inner_side)
	//  	cout << "intersecting particle found with inner side" << endl;
	// if (outer_side)
	//	cout << "intersecting particle found with outer side" << endl;
	//if (front_side)
	//	cout << "intersecting particle found with front side" << endl;

    // float theta = M_PI / 2;
    // if (path_option.obstruction != Eigen::Vector2f::Zero()) {
    //     theta = curvature < 0 ? atan2(path_option.obstruction[0], path_option.obstruction[1]- c[1]) :
    //         atan2(path_option.obstruction[0], c[1] - path_option.obstruction[1]);
    // }
    // clearance
    // path_option.clearance = 100; // some large number
    for (auto p: point_cloud) {
        float theta_p =  curvature < 0 ? atan2(p[0], p[1]- c[1]) :
            atan2(p[0], c[1] - p[1]);
        float path_len_p = theta_p * (p-c).norm();
        if (path_len_p >=0 and path_len_p < path_option.free_path_length) {  // if p is within the fp length
            float inner = abs((c - p).norm() - r_inner);
            float outer = abs((c - p).norm() - r_tr);
            float clearance_p = std::min(inner, outer);

            if (clearance_p < path_option.clearance) {
                path_option.clearance = clearance_p;
                // path_option.closest_point = p;
            }
        }
    }

}


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

PathCandidate Controller::evaluatePaths(const std::vector<Vector2f>& point_cloud, std::vector<PathCandidate> &candidates)
{
  // creating starting path (with terrible score)
  auto best_path {PathCandidate(-100)};

  // weights
  // float w1{8.f}, w2{-0.5f};
  // float w1{10.f}, w2{-0.5f};
  float w_fpl {1.0f}, w_clr {0.1}, w_dst {0.0}, w_dev {0.0};
  
  // Evaluate all possible paths and select optimal option
  for (float path_curvature = -1 * (car_->limits_.max_curvature_); path_curvature <= car_->limits_.max_curvature_; path_curvature += curvature_sampling_interval_) {
    
    // create candidate for this path
    auto candidate {PathCandidate(-100)};
    candidate.free_path_length = 10;
    candidate.clearance = 10;
    // & Replaced by path candidate
    // candidate.curvature = path_curvature;

    // // calculate free path length
    // candidate.free_path_length = Controller::calculateFreePathLength(point_cloud, candidate.curvature);
    
    // // calculate clearance
    // // candidate.clearance = Controller::calculateClearance(point_cloud, candidate.curvature, candidate.free_path_length);
    // // candidate.clearance = 0.5; // & setting clearance to a max vakue
    // Controller::calculateClearance(point_cloud, candidate);

    Controller::computePathOption(candidate, path_curvature, point_cloud);

    // goal distance metric
    candidate.goal_distance = Controller::calculateDistanceToGoal(candidate.curvature);

    // similarity to previous path metric
    candidate.deviance = std::abs(candidate.curvature - previous_curvature_);

    // Calculate score and update selection
    candidate.score = w_fpl * candidate.free_path_length + w_clr * candidate.clearance + w_dst * candidate.goal_distance + w_dev * candidate.deviance;

    // Add path to candidates
    candidates.push_back(candidate);
    
    if (candidate.score > best_path.score) {
      best_path = candidate;
    }
  }

  // for (const auto &path : candidates) {
  //   std::cout << "\t" << path.clearance << std::endl;
  // }

  // . Now iterate over them with window and calculate the scores (using neighbors to avoid taking extreme dangerous paths)
  // const int n_neighbors_unilateral {2};
  // int best_path_index {-1};

  // for (std::size_t i = 0 + n_neighbors_unilateral; i < candidates.size() - n_neighbors_unilateral + 1; i++) {
  //   // - compose lists
  //   std::vector<PathCandidate> neighbor_paths;
  //   neighbor_paths.reserve(2 * n_neighbors_unilateral + 1);
  //   // std::cout << "\t\tReserved " << 2 * n_neighbors_unilateral + 1 << " spots in memory for vector...\n";
  //   // int counter {};
  //   for (std::size_t j = i - n_neighbors_unilateral; j < i + n_neighbors_unilateral + 1; j++) {
  //     neighbor_paths.push_back(candidates[i]);
  //     // counter++;
  //   }
  //   // std::cout << "\t\t\t...added to it " << counter << " times." << std::endl;

  //   // - compute avg path
  //   auto avg_path {PathCandidate(neighbor_paths)};

  //   // - compute score
  //   // candidates[i] = avg_path;
  //   candidates[i].score = w_fpl * avg_path.free_path_length + w_clr * avg_path.clearance + w_dst * avg_path.goal_distance + w_dev * avg_path.deviance;

  //   // - compare and set if best
  //   if (candidates[i].score > candidates[best_path_index].score) {
  //     best_path_index = static_cast<int>(i);
  //   }
  // }
  // best_path = candidates[best_path_index];

  // float deviance {std::abs(previous_curvature_ - best_path.curvature)};
  // previous_curvature_ = best_path.curvature;

  // // . Instead trying to sort candidates
  // std::sort(candidates.begin(), candidates.end());
  // std::cout << "front:" << "\n";
  // for (const auto & candidate : candidates) {
  //   std::cout << "\t" << candidate.score << "\t" << candidate.curvature << "\t" << candidate.free_path_length <<  "\n";
  // }

  // // . accumulate all paths within some tolerance
  // std::vector<PathCandidate> shortlist {candidates[0]};
  // for (std::size_t i = 1; i < candidates.size(); i++) {
  //   if (std::abs(candidates[0].score - candidates[i].score) / candidates[0].score < 0.05) {
  //     shortlist.push_back(candidates[i]);
  //   }
  // }
  // std::cout << "Shortlist has " << shortlist.size() << " entries." << std::endl;
  
  // . now pick the element from the shortlist that is most similar to the previous commanded curvature
  // std::size_t closest_index {0};
  // float closest_curvature {shortlist[0].curvature};
  // for (std::size_t i = 1; i < shortlist.size(); i++) {
  //   if (shortlist[i].curvature < closest_curvature) {
  //     closest_index = i;
  //     closest_curvature = shortlist[i].curvature;
  //   }
  // }
  // best_path = shortlist[closest_index];
  // best_path = shortlist[0];

  // std::cout << "C:" << best_path.curvature << "\tFPL:" << best_path.free_path_length << "\tCLR:" << best_path.clearance << "\tDST:" << best_path.goal_distance << "\tDEV:" << best_path.deviance << std::endl;
  return best_path;
}

Command Controller::generateCommand(const std::vector<Vector2f>& point_cloud, const float current_speed, std::vector<PathCandidate> &path_candidates, PathCandidate &best_path)
{
  PathCandidate path {Controller::evaluatePaths(point_cloud, path_candidates)};
  best_path = path;
  float speed {Controller::calculateControlSpeed(current_speed, path.free_path_length)};
  // std::cout << "FPL: " << path.free_path_length << ", " << "Current speed: " << current_speed << ", " << "Commanded speed: " << speed << std::endl;
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

time_optimal_1D::Command Controller::generateCommand(const std::vector<Vector2f>& point_cloud, const float current_speed, const double last_data_timestamp, std::vector<PathCandidate> &path_candidates, PathCandidate &best_path)
{
  // using latency, and history of results, project the car's position and velocity forward through time; search the controls queue and pop until a timestamp is newer than the current time
  State2D projected_state {Controller::projectState(current_speed, last_data_timestamp)};

  // use this forward projection to transform the point cloud
  auto cloud {Controller::transformCloud(point_cloud, projected_state)};

  // feed these updated parameters into the 1D time-optimal controller
  time_optimal_1D::Command command {toc_->generateCommand(cloud, projected_state.speed, path_candidates, best_path)};

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
