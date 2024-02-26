//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    particle-filter.cc
\brief   Particle Filter Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

#include <queue>

using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;
using math_util::AngleDiff;

DEFINE_double(pi, 3.1415926, "Pi");
//TODO This shouldnt be a double :)
DEFINE_double(num_particles, 50, "Number of particles");
// TODO This noise parameters will need to be tuned.
DEFINE_double(k1, 0.2, "Error in translation from translation motion");
DEFINE_double(k2, 0.2, "Error in rotation from translation motion");
DEFINE_double(k3, 0.2, "Error in rotation from rotation motion");
DEFINE_double(k4, 0.2, "Error in translation from rotation motion");

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false),
    resampling_iteration_counter_(0) {}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr) {
  vector<Vector2f>& scan = *scan_ptr;
  //*Look at GetPredictedScan in vector_map.cc

  



  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.
  // TODO There are 1081 lasers, I remember speaking in class about reducing this number for improved performance. Here I'm reducing by a factor of 5, adjust number if needed in scan.resize() angle_increment = ...!
  // Will this reduction come into play at a later stage?

  // The returned values must be set using the 'scan' variable:
  const int laser_downsampling_factor {5};
  scan.resize(num_ranges / laser_downsampling_factor);

  // Calculate lidar location (0.2m in front of base_link)
  Vector2f lidar_loc = loc + 0.2 * Vector2f(cos(angle), sin(angle));

  // std::cout << loc << "  " << angle << "  " << num_ranges << "  " << angle_min << "  " << angle_max << std::endl;

  // Loop through laser scans creating a line for each ray
  float angle_increment = (angle_max - angle_min) / num_ranges * laser_downsampling_factor;
  for (size_t i = 0; i < scan.size(); i++) {
    // Calculate angle of the ray
    float ray_angle = angle + angle_min + i * angle_increment;

    // Create a line for the ray
    line2f ray(0.0, 0.0, 1.0, 1.0);   // line from segment from (0,0) to (1,1)
    ray.p0.x() = lidar_loc.x() + range_min * cos(ray_angle);
    ray.p0.y() = lidar_loc.y() + range_min * sin(ray_angle);
    ray.p1.x() = lidar_loc.x() + range_max * cos(ray_angle);
    ray.p1.y() = lidar_loc.y() + range_max * sin(ray_angle);

    // Laserscan maximum value is default ray intersection with the map
    Vector2f ray_intersection = lidar_loc + range_max * Vector2f(cos(ray_angle), sin(ray_angle));
    float obstacle_dist = range_max;

    // Loop through map lines checking for intersections with the predicted ray
    for (size_t j = 0; j < map_.lines.size(); ++j) {
      const line2f map_line = map_.lines[j];

      // Check for intersection between ray and map line
      Vector2f intersection_point;
      bool intersects = map_line.Intersection(ray, &intersection_point);

      // Update if the intersection is closer (deal with multiple collisions, take first obstacle seen)
      if (intersects && (intersection_point - loc).norm() < obstacle_dist) {
        obstacle_dist = (intersection_point - loc).norm();    // Update distance
        ray_intersection = intersection_point;    // Update location
      }
    }

    // Lastly, add obstacle to generated scan
    // std::cout << ray_intersection.transpose() << std::endl;
    scan[i] = ray_intersection;
  }

}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  //TODO Multiply previous weight?
  //TODO Normalize to wmax
  //TODO Lecture 08 slide 44, Lecture 7 slide 32, log likelihoods and infitesimally small numbers

  //! D_long and D_short to be tuned!!! 
  //! Gamma is tuned to reduce overconfidence
  //TODO All of these must be tuned. 
  float d_long = 1.0;
  float d_short = 0.5;
  float sigma_s = 0.03; //Intuition was correct, look for lidar spec sheet!! 
  float gamma = 0.1; //Note: Can range from 1/1081 to 1 (or is it 1/#particles?)
  


  vector<Vector2f> predicted_scan; //This scan will be altered by GetPredictedPointCloud to be compared to ranges
  Particle particle = *p_ptr;
  GetPredictedPointCloud(particle.loc, particle.angle, ranges.size(), range_min, range_max, angle_min, angle_max, &predicted_scan);

  int downsampling_factor {static_cast<int>(ranges.size() / predicted_scan.size())};
  std::vector<float> downsampled_ranges(predicted_scan.size());
  for (std::size_t i = 0; i < predicted_scan.size(); i++) {
    downsampled_ranges[i] = ranges[i * downsampling_factor];
  }

  float p = 0;

  const Eigen::Vector2f kLaserLoc(0.2, 0); // pulled from navigation_main.cc
  Eigen::Vector2f lidar_location = particle.loc + kLaserLoc(0) * Eigen::Vector2f(cos(particle.angle), sin(particle.angle));
  for (size_t i=0; i<predicted_scan.size(); i++){

    double predicted_scan_range {(predicted_scan[i] - lidar_location).norm()}; 

    // todo add comment
    if (downsampled_ranges[i] < range_min || downsampled_ranges[i] > range_max){
      continue;
    }
    if (downsampled_ranges[i] < predicted_scan_range-d_short){
      p += (-(d_short*d_short)/(sigma_s*sigma_s));
    }
    else if (downsampled_ranges[i] > predicted_scan_range+d_long){
      p += (-(d_long*d_long)/(sigma_s*sigma_s));
    }
    else{
    p += (-pow((downsampled_ranges[i]-predicted_scan_range),2)/(pow(sigma_s,2)));
    }
  }
  p_ptr->weight = p*gamma;
}

void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights.
  // The current particles are in the `particles_` variable. 
  // Create a variable to store the new particles, and when done, replace the
  // old set of particles:
  // vector<Particle> new_particles';
  // During resampling: 
  //    new_particles.push_back(...)
  // After resampling:
  // particles_ = new_particles;

  std::cout << "\tResampling particles" << std::endl;

  // checking to see if we have anything to resample
  if (particles_.empty()) {
      std::cerr << "NO EXISTING PARTICLES! Nothing to resample..." << std::endl;
      return;
  }

  // create vector for new particles
  const int n_particles {50}; //TODO: Jared asks is there a reason why this isn't equal to FLAGS_num_particles?
  std::vector<particle_filter::Particle> resampled_particles;
  resampled_particles.reserve(n_particles);

  // get max weight for normalization
  double max_particle_weight {particles_[0].weight};
  if (static_cast<int>(particles_.size()) > 1) {
    for (std::size_t i = 1; i < particles_.size(); i++) {
      if (particles_[i].weight > max_particle_weight) {
        max_particle_weight = particles_[i].weight;
      }
    }
  }
  
  // normalize and build discrete distribution for resampling
  double weights_sum {0.d};
  std::queue<double> relative_positions;
  for (std::size_t i = 0; i < particles_.size(); i++) {
    particles_[i].weight -= max_particle_weight; // normalizing the particle's weight
    weights_sum += exp(particles_[i].weight); // converting this out of log space to get the range for low-variance resampling
    relative_positions.push(weights_sum); // keeping track of where the particles fall in the distribution we are going to resample for convenince
  }

  // create starting point and step size for low-variance resampling
  // double r {rng_.UniformRandom(0, weights_sum / n_particles)};
  double low_variance_sampling_step {weights_sum / n_particles}; //
  double current_sampling_location {rng_.UniformRandom(0, low_variance_sampling_step)}; // starting point for low-variance resampling; constraining it to be within the first step so we don't have to worry about wrapping around

  // resample
  int original_particle_counter {0}; // to track which particle from the original vector to add to the resampled vector
  for (std::size_t i = 0; i < n_particles; i++) {
    while (!relative_positions.empty()) {
      if (relative_positions.front() < current_sampling_location) {
        relative_positions.pop();
        original_particle_counter++;
      }
      current_sampling_location += low_variance_sampling_step;
      resampled_particles.push_back(particles_[original_particle_counter]);
      break; // to make sure we only increment once for each iteration
    }
  }

  // make sure we resampled how many particles we wanted
  if (static_cast<int>(resampled_particles.size()) != n_particles) {
    std::cerr << resampled_particles.size() << " particles were resampled instead of " << n_particles << "! Investigate this." << std::endl;
  }

  // for (auto particle : particles_) {
  //   std::cout << "Resampled: " << particle.weight << std::endl;
  // }

  // ? should all the particles be unweighted now? Should they be explicitly set to some value or just kept as is here?
  // ! I'm going to disable this for now so we can use the weights in the GetLocation function, but maybe this is incorrect
  // double resampled_weight {log(1.d / n_particles)};
  // for (auto &particle : resampled_particles) {
  //   particle.weight = resampled_weight;
  // }

  // assign the resampled particles
  particles_ = resampled_particles;
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {

  // TODO we actually need to make the distance traveled check happen here
  // ! record the distance we last updated at? but we shouldn't be predicting either? so this should actually go elsewhere

  // Ignore callback when odometry has not changed 
  if (!odometry_flag_) {
    return;
  }

  //Note: If sensor data is available *and car has travelled at least distance d*
  for (auto &particle : particles_) {
    Update(ranges, range_min, range_max, angle_min, angle_max, &particle);
  }
  std::cout << " " << std::endl;

  // -For every particle: Calculate the weight of said particle using the update function to compare the expected pointcloud to the viewed pointcloud
  // Call the Update function to update the weights for all the particles based on their observation likelihood and the latest laser scan.
  // -Trim bad particles and duplicate good particles, according to weights (?). Only do this ever N observations. 
  // Call the Resample function to update the particle cloud. This will remove unlikely particles, keep likely ones, and add particles closer to true location if necessary.
  // Lastly, maintain the pose of the particle with the highest weight.(this may be implemented in GetLocation() and we might want to keep a variable with that pose).

  // check how many iterations we have had since resampling, resample if it's time to do so
  const int resampling_iteration_threshold {10};
  resampling_iteration_counter_++;
  if (resampling_iteration_counter_ == resampling_iteration_threshold) {
    resampling_iteration_counter_ = 0;
    Resample();
  }
  // this should not be called here // GetLocation();
}

// A new odometry value is available. Propagate the particles forward using the motion model.
void ParticleFilter::Predict(const Vector2f& odom_loc,
                             const float odom_angle) {
  // Ignore new pose set
  if (odom_initialized_) {
    // Calculate pose change from odometry reading
    Vector2f translation_diff = odom_loc - prev_odom_loc_;
    float rotation_diff = odom_angle - prev_odom_angle_;

    // Set flag if odometry change
    if (translation_diff.norm() < 0.005 && rotation_diff < 0.005) {
      odometry_flag_ = false;
      return;
    }
    else {
      odometry_flag_ = true;
    }

    // Ignore unrealistic jumps in odometry
    if (translation_diff.norm() < 1.0 && abs(rotation_diff) < FLAGS_pi / 4) {
      // Loop through particles
      for (auto &particle : particles_) {
        // Transform odometry pose change to map frame (for particle)
        Eigen::Rotation2Df rotation_vector(AngleDiff(particle.angle, prev_odom_angle_));
        Vector2f particle_translation = rotation_vector * translation_diff;

        // Sample noise from a Gaussian distribution
        float x_noise = rng_.Gaussian(0.0,  FLAGS_k1 * translation_diff.norm() + FLAGS_k4 * rotation_diff);
        float y_noise = rng_.Gaussian(0.0,  FLAGS_k1 * translation_diff.norm() + FLAGS_k4 * rotation_diff);
        float rotation_noise = rng_.Gaussian(0.0, FLAGS_k2 * translation_diff.norm() + FLAGS_k3 * rotation_diff);

        // std::cout << x_noise << ", " << y_noise << ", " << rotation_noise << std::endl;

        // Update particle location from motion model
        particle.loc[0] += particle_translation[0] + x_noise;
        particle.loc[1] += particle_translation[1] + y_noise;
        particle.angle += rotation_diff + rotation_noise;
      }
    }
  }
  else {
    prev_odom_loc_ = odom_loc;
    prev_odom_angle_ = odom_angle;
    odom_initialized_ = true;
  }

  // Update previous odometry
  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  map_.Load(map_file);

  // Initialize odometry
  prev_odom_loc_ = loc;
  prev_odom_angle_ = angle;
  odom_initialized_ = false;

  // Clear and Initialize particles
  particles_.clear();
  for (unsigned i = 0; i < FLAGS_num_particles; i++) {
    // Generate random number for error
    float error_x = rng_.UniformRandom(-0.25, 0.25);  
    // float error_x = 0.0;
    float error_y = rng_.UniformRandom(-0.25, 0.25);
    float error_theta = rng_.UniformRandom(-FLAGS_pi / 6, FLAGS_pi / 6);
    // float error_theta = 0.0;

    // Create particle using error weights
    Particle p = {
      Eigen::Vector2f(loc[0] + error_x, loc[1] + error_y),
      (float)(angle + error_theta),
      1 / FLAGS_num_particles};

    // Add particle
    particles_.push_back(p);
  }
  // Particle perf = {
  //     Eigen::Vector2f(loc[0], loc[1]),
  //     (float)(angle),
  //     1 / FLAGS_num_particles};
  // particles_.push_back(perf);
  // particles_.push_back(perf);
  std::cout << particles_.size() << " particles initialized" << std::endl;
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:
  // loc = Vector2f(0, 0);
  // angle = 0;

  // - two ideas here:
  // 1) just take the highest-probability particle (this is probably the way to go )
  // 2) use a weighted average of the particles

  // ! actually, neither of these solutions is possible if the weights are all set to 1/N at the end of the resampling step; for now I'm going to keep the weight on each particle (not reset them to 1/N at the end of Resample) so they can be used here (maybe this won't cause any issues since the weights are reset from zero in the update step anyway?)
  // get the most likely particle
  int most_likely_particle_index {0};
  double most_likely_particle_weight {particles_[0].weight};
  for (std::size_t i = 1; i < particles_.size(); i++) {
    if (particles_[i].weight > most_likely_particle_weight) {
      most_likely_particle_index = static_cast<int>(i);
      most_likely_particle_weight = particles_[i].weight;
    }
  }

  // - for just returning the most likely particle
  // loc = particles_[most_likely_particle_index].loc;
  // angle = particles_[most_likely_particle_index].angle;

  // but maybe we should use the particles close to the single most likely one?
  double radial_inclusion_distance {0.5}; // m
  auto most_likely_location {particles_[most_likely_particle_index].loc};

  auto location_estimate {particles_[most_likely_particle_index].loc};
  auto angle_estimate {particles_[most_likely_particle_index].angle};
  int total_considered_particles {1};

  double radial_inclusion_distance_sqrd {pow(radial_inclusion_distance, 2)};
  for (std::size_t i = 0; i < particles_.size(); i++) {
    
    // calculate the distance from the most likely location to the particle
    double radial_distance_sqrd {pow(particles_[i].loc.x() - most_likely_location.x(), 2) + pow(particles_[i].loc.y() - most_likely_location.y(), 2)};

    // don't consider points farther from the most likely particle than the set distance
    if (radial_distance_sqrd > radial_inclusion_distance_sqrd) {continue;}

    // accumulate the point for calculations (probably start with a simple average, then maybe can consider a probability- or distance-weighted average depending on the results)
    total_considered_particles++;
    location_estimate.x() += particles_[i].loc.x();
    location_estimate.y() += particles_[i].loc.y();
    angle_estimate += particles_[i].angle;
  }

  // divide to get averages
  location_estimate.x() /= total_considered_particles;
  location_estimate.y() /= total_considered_particles;
  angle_estimate /= total_considered_particles;

  loc = location_estimate;
  angle = angle_estimate;
  std::cout << "The best particle is at: " << loc << std::endl;
}

}  // namespace particle_filter
