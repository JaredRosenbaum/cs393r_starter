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
    odom_initialized_(false) {}

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
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.

  // // Note: The returned values must be set using the `scan` variable:
  // scan.resize(num_ranges);
  // // Fill in the entries of scan using array writes, e.g. scan[i] = ...
  // for (size_t i = 0; i < scan.size(); ++i) {
  //   scan[i] = Vector2f(0, 0);
  // }

  // // The line segments in the map are stored in the `map_.lines` variable. You
  // // can iterate through them as:
  // for (size_t i = 0; i < map_.lines.size(); ++i) {
  //   const line2f map_line = map_.lines[i];
  //   // The line2f class has helper functions that will be useful.
  //   // You can create a new line segment instance as follows, for :
  //   line2f my_line(1, 2, 3, 4); // Line segment from (1,2) to (3.4).
  //   // Access the end points using `.p0` and `.p1` members:
  //   printf("P0: %f, %f P1: %f,%f\n", 
  //          my_line.p0.x(),
  //          my_line.p0.y(),
  //          my_line.p1.x(),
  //          my_line.p1.y());

  //   // Check for intersections:
  //   bool intersects = map_line.Intersects(my_line);
  //   // You can also simultaneously check for intersection, and return the point
  //   // of intersection:
  //   Vector2f intersection_point; // Return variable
  //   intersects = map_line.Intersection(my_line, &intersection_point);
  //   if (intersects) {
  //     printf("Intersects at %f,%f\n", 
  //            intersection_point.x(),
  //            intersection_point.y());
  //   } else {
  //     printf("No intersection\n");
  //   }
  // }

  // TODO There are 1081 lasers, I remember speaking in class about reducing this number for improved performance. Here I'm reducing by a factor of 5, adjust number if needed in scan.resize() angle_increment = ...!
  // Will this reduction come into play at a later stage?

  // The returned values must be set using the 'scan' variable:
  scan.resize(num_ranges / 5);

  // Calculate lidar location (0.2m in front of base_link)
  Vector2f lidar_loc = loc + 0.2 * Vector2f(cos(angle), sin(angle));

  // std::cout << loc << "  " << angle << "  " << num_ranges << "  " << angle_min << "  " << angle_max << std::endl;

  // Loop through laser scans creating a line for each ray
  float angle_increment = (angle_max - angle_min) / num_ranges * 5;
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
    scan[i] = ray_intersection;
  }
}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  // Implement the update step of the particle filter here.
  // You will have to use the `GetPredictedPointCloud` to predict the expected
  // observations for each particle, and assign weights to the particles based
  // on the observation likelihood computed by relating the observation to the
  // predicted point cloud.

  //Note: Lecture 08, slides around 38
  //note: The weight of a particle just p(s|x), or the sum of p(s|x) for each beam.
  //TODO Work with log likelihood? Lecture 7 slide 32
  //TODO Multiply previous weight?
  // TODO Pseudo Code ideas:
  //note observed reading is s^, expected is s
  //! D_long and D_short to be tuned!!! 
  float d_long = 1.0;
  float d_short = 0.2;
  //!What is sigma s?
  float sigma_s = 1.0;
  // .Input is the current laserscan, and the particle we want to compare it to.
  // .For that particle, get the predicted point cloud.
  vector<Vector2f> scan; //This scan will be altered by GetPredictedPointCloud to be compared to ranges`
  Particle particle = *p_ptr;
  Eigen::Vector2f particle_loc = particle.loc;
  float p_ang = particle.angle;
  int scan_lasers = 1081; //TODO Where can we pull this from? scan.size?
  GetPredictedPointCloud(particle_loc, p_ang, scan_lasers, range_min, range_max, angle_min, angle_max, &scan);
  // .For each beam in that point cloud, compare it to the current laserscan. 
  // .Each beam's p should be calculated with the equation on slide 38
  float p = 0;
  for (size_t i=0; i<scan.size(); i++){
    if (scan[i].norm() < range_min || scan[i].norm() > range_max){
      p += 0;
    }
    else if (scan[i].norm() < ranges[i]-d_short){
      p += exp(-(d_short*d_short)/(sigma_s*sigma_s));
    }
    else if (scan[i].norm() > ranges[i]+d_long){
      p += exp(-(d_long*d_long)/(sigma_s*sigma_s));
    }
    else{
      p += exp(-pow((scan[i].norm()-ranges[i]),2)/(pow(sigma_s,2)));
    }
  }
  // .Sum these p's, that is the weight for the specific particle. Perhaps normalization is necessary? Maybe to # of beams. 
  p_ptr->weight = p;

  // Loop through particle cloud.
  // - GetPredictedPointCloud
  // - Implement some logic to assign weights compared to the newly received laser scan. Maybe something with a log likelihood whatever that means mathematically.
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

  // You will need to use the uniform random number generator provided. For
  // example, to generate a random number between 0 and 1:
  float x = rng_.UniformRandom(0, 1);
  printf("Random number drawn from uniform distribution between 0 and 1: %f\n",
         x);


  // TODO Presudo Code ideas:
  // Create a distribution from the given particle cloud (i.e. we have 50 particles with different weights)
  // Sample from this distribution (do not keep duplicates). We should now have a smaller cloud with only more likely particles.
  // Loop through max num of particles (FLAGS_num_particles)
  // - Generate new particles to fill up for the removed ones based on our current best location estimate (best location as of our last update).
  // - Push the particles to the particles_ list.
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.


  // TODO Pseudo Code ideas:
  for (auto &particle : particles_) {
    Update(ranges, range_max, range_max, angle_max, angle_max, &particle);
  }

  // -For every particle: Calculate the weight of said particle using the update function to compare the expected pointcloud to the viewed pointcloud
  // Call the Update function to update the weights for all the particles based on their observation likelihood and the latest laser scan.
  // -Trim bad particles and duplicate good particles, according to weights (?). Only do this ever N observations. 
  // Call the Resample function to update the particle cloud. This will remove unlikely particles, keep likely ones, and add particles closer to true location if necessary.
  // Lastly, maintain the pose of the particle with the highest weight.(this may be implemented in GetLocation() and we might want to keep a variable with that pose).
}

// A new odometry value is available. Propagate the particles forward using the motion model.
void ParticleFilter::Predict(const Vector2f& odom_loc,
                             const float odom_angle) {
  // Implement the predict step of the particle filter here.
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.

  // You will need to use the Gaussian random number generator provided. For
  // example, to generate a random number from a Gaussian with mean 0, and
  // standard deviation 2:
  // float x = rng_.Gaussian(0.0, 2.0);
  // printf("Random number drawn from Gaussian distribution with 0 mean and "
  //        "standard deviation of 2 : %f\n", x);

  // Ignore new pose set
  if (odom_initialized_) {
    // Calculate pose change from odometry reading
    Vector2f translation_diff = odom_loc - prev_odom_loc_;
    float rotation_diff = odom_angle - prev_odom_angle_;

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
    float error_y = rng_.UniformRandom(-0.25, 0.25);
    float error_theta = rng_.UniformRandom(-FLAGS_pi / 6, FLAGS_pi / 6);

    // Create particle using error weights
    Particle p = {
      Eigen::Vector2f(loc[0] + error_x, loc[1] + error_y),
      (float)(angle + error_theta),
      1 / FLAGS_num_particles};

    // Add particle
    particles_.push_back(p);
  }
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:
  loc = Vector2f(0, 0);
  angle = 0;
}


}  // namespace particle_filter
