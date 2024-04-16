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
\file    slam.cc
\brief   SLAM Starter Code
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
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "slam.h"
#include "parameters.h"

#include "vector_map/vector_map.h"

using namespace math_util;
using Eigen::Affine2f;
using Eigen::Rotation2Df;
using Eigen::Translation2f;
using Eigen::Vector2f;
using Eigen::Vector2i;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using vector_map::VectorMap;

namespace slam {

SLAM::SLAM() :
    odom_initialized_(false),
    motion_model_ready_(false) {}

void SLAM::CreateVisPublisher(ros::NodeHandle* n) {
    vis_pub_ = n->advertise<amrl_msgs::VisualizationMsg>("visualization", 1);
    vis_msg_ = visualization::NewVisualizationMessage("map", "slam");
}

// TODO This method was added to work with map frame. Repurpose it for visualization purposes with helped functions.
void SLAM::InitializePose(const Eigen::Vector2f& loc, const float angle) {
  current_pose_.loc = loc;
  current_pose_.angle = angle;

  odom_initialized_ = false;
}

void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
  // Return the latest pose estimate of the robot.
  *loc = current_pose_.loc;
  *angle = current_pose_.angle;
}

void SLAM::ObserveLaser(const vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max) {
  // A new laser scan has been observed. Decide whether to add it as a pose
  // for SLAM. If decided to add, align it to the scan from the last saved pose,
  // and save both the scan and the optimized pose.

  // TODO should the laser scan be downsampled?

  // Disregard scans until motion model threshold has been met
  if (motion_model_ready_) {

    // Convert laser scan to point cloud centered around base_link
    float angle_inc = (angle_max - angle_min) / ranges.size();
    current_point_cloud_.clear();
    for (std::size_t i = 0; i < ranges.size(); i++) {
      // Ignore points that are not obstacles
      if (ranges[i] >= range_max) {
        continue;
      }

      // Create point coordinates in robot frame
      const Eigen::Vector2f lidar_loc(0.2, 0);    // LiDAR is 20cm in front of base link
      float theta = angle_inc * i + angle_min;    // angle_min is -2.35619
      Eigen::Vector2f point(
        ranges[i] * cos(theta) + lidar_loc.x(),
        ranges[i] * sin(theta) + lidar_loc.y()
      );

      // Push into point cloud vector
      current_point_cloud_.push_back(point);

      // Draw point visualization
      visualization::DrawPoint(point, 0x5de053, vis_msg_);
    }

    // Publish point cloud visualization
    vis_msg_.header.stamp = ros::Time::now();
    vis_pub_.publish(vis_msg_);

    // Proceed with transforming laser scan into every candidate of the motion model
    if(!previous_point_cloud_.empty()){
      ConfigureCandidates(current_point_cloud_, previous_point_cloud_);
    }

    // TODO: Should this occur here or be moved?
    // Select best candidate
    double best_score {0.d};
    Candidate best_cand;
    for (auto &candidate : candidates_){
      if (candidate.p_motion*candidate.p_scan > best_score){
        best_score = candidate.p_motion*candidate.p_scan;
        best_cand = candidate;
      }
    }
    // Populate state
    // State selected_state;
    // selected_state.pose = best_cand.pose;

    // //! is there a better way to populate this pointcloud?
    // Eigen::Rotation2Df rotation_transform(best_cand.pose.angle);
    // std::vector<Eigen::Vector2f> transformed_point_cloud;
    // for (const auto &point : current_point_cloud_) {
    //   Eigen::Vector2f transformed_point = rotation_transform * point + best_cand.pose.loc;
    //   transformed_point_cloud.push_back(transformed_point);
    // }
    // selected_state.point_cloud = transformed_point_cloud;

    //TODO JARED
    // selected_state.cov = TODO;

    // state_chain_.push_back(selected_state);

    // With the candidates configured, archive the current point cloud to be used at the next step
    previous_point_cloud_ = (current_point_cloud_);

    // StateTransform(3);


    // Clear motion model flag
    motion_model_ready_ = false;
  }
}

void SLAM::ConfigureCandidates(const std::vector<Eigen::Vector2f> &point_cloud, const std::vector<Eigen::Vector2f> &stored_point_cloud) {
  // Loop through each candidate configuring their probabilites
  const float sigma {0.1f};
  float fine_resolution {0.03}; 
  const auto lookup_table {std::make_unique<rasterization::LookupTable>(stored_point_cloud, fine_resolution, sigma)};
  // Note: Save to make sure it looks good, can comment
  lookup_table->exportAsPPM("/home/jared/CS393/cs393r_starter/images/lookup_table.ppm"); //TODO workstation changes
  // lookup_table->exportAsPPMRandom("/home/jared/CS393/cs393r_starter/images/pixels_coarse_random.ppm");
  // for (const auto &point : point_cloud){
  //   visualization::DrawPoint(point, 0xfcba03, vis_msg_);
  // }

  for (auto &candidate : candidates_) {
    // Candidate pose rotation
    Eigen::Rotation2Df rotation_transform(candidate.pose.angle);

    // Loop through point cloud transforming it to candidate frame
    std::vector<Eigen::Vector2f> candidate_point_cloud;
    for (const auto &point : point_cloud) {
      Eigen::Vector2f transformed_point = rotation_transform * point + candidate.pose.loc;
      candidate_point_cloud.push_back(transformed_point);
      visualization::DrawPoint(transformed_point, 0x536de0, vis_msg_);
    }
    
    for (const auto &point : stored_point_cloud){
      visualization::DrawPoint(point, 0xfcba03, vis_msg_);
    }

    //. Observation model
    // p(z|x_i,m)
    double score {};
    for (const auto &point : candidate_point_cloud) {
      // Log probabilities
      score += (lookup_table->lookupUnnormalizedProbability(point));
      //? Gamma for confidence?

    }
    candidate.p_scan = score/candidate_point_cloud.size();
    //. Motion model
    // p(x_i|x_i-1,u)
    std::cout << candidate.pose.loc.transpose() << " " << candidate.pose.angle << " " << candidate.p_scan << std::endl;

    
    // Publish point cloud visualization
    vis_msg_.header.stamp = ros::Time::now();
    vis_pub_.publish(vis_msg_);
    std::cout << "AAAA";
    std::cin.get();

    visualization::ClearVisualizationMsg(vis_msg_);


  }
}

void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {

  // std::cout << odom_loc.x() << "\t" << odom_loc.y() << "\t" << odom_angle << std::endl;

  // Disregard first odometry reading, set pose variables
  if (!odom_initialized_) {
    reference_odom_pose_.loc = odom_loc;
    reference_odom_pose_.angle = odom_angle;

    odom_initialized_ = true;
  }
  else {
    // Calculate pose change from odometry reading
    Eigen::Vector2f odom_translation = odom_loc - prev_odom_pose_.loc;
    float odom_rotation = AngleDiff(odom_angle, prev_odom_pose_.angle);

    // Ignore unrealistic jumps in odometry
    if (odom_translation.norm() < 1.0 && abs(odom_rotation) < M_PI_4) {

      // Keep track of pose change from odometry to estimate how far the robot has moved.
      float translation = (odom_loc - reference_odom_pose_.loc).norm();
      float rotation = AngleDiff(odom_angle, reference_odom_pose_.angle);

      // Proceed with preparing a motion model when meeting threshold
      if ((translation > ODOM_TRANSLATION_THRESHOLD ||
        std::abs(rotation) > ODOM_ROTATION_THRESHOLD) && !motion_model_ready_) {
        // Calculate the change in odometry in reference to the odom frame (reference odom pose is (0, 0, 0))
        Pose odom_change;
        odom_change.loc = Eigen::Vector2f(
          translation * cos(rotation),
          translation * sin(rotation)
        );
        odom_change.angle = rotation;

        PrepareMotionModel(odom_change);

        // Update reference pose
        // TODO Is this independent of the CSM? Or will it get updated later?
        reference_odom_pose_.loc = odom_loc;      
        reference_odom_pose_.angle = odom_angle;
      }
    }
  }

  // Update previous odometry reading
  prev_odom_pose_.loc = odom_loc;
  prev_odom_pose_.angle = odom_angle;
}

void SLAM::PrepareMotionModel(const Pose odom_change) {
  // Clear particles visualization
  visualization::ClearVisualizationMsg(vis_msg_);

  // Clear state candidates vector
  candidates_.clear();

  // Calculate number of divisions (1/2 loop iterations) in each dimension x, y, theta
  const static int x_iterations = std::ceil(MOTION_MODEL_X_LIMIT / MOTION_MODEL_X_RESOLUTION);
  const static int y_iterations = std::ceil(MOTION_MODEL_Y_LIMIT / MOTION_MODEL_Y_RESOLUTION);
  const static int theta_iterations = std::ceil(MOTION_MODEL_THETA_LIMIT / MOTION_MODEL_THETA_RESOLUTION);

  // Triple loop to populate motion model poses with variance in each dimension. This creates a cube of next state possibilities
  for (int theta_i = -theta_iterations; theta_i <= theta_iterations; theta_i++) {
    for (int  y_i = -y_iterations; y_i <= y_iterations; y_i++) {
      for (int x_i = -x_iterations; x_i <= x_iterations; x_i++) {
        // Create pose candidate in reference to the previous state (which is always at origin)
        Pose candidate_pose;
        candidate_pose.loc = odom_change.loc + Eigen::Vector2f(
          x_i * MOTION_MODEL_X_RESOLUTION,
          y_i * MOTION_MODEL_Y_RESOLUTION
        );
        candidate_pose.angle = odom_change.angle + theta_i * MOTION_MODEL_THETA_RESOLUTION;

        // Push a new candidate into the candidates vector
        Candidate candidate;
        candidate.pose = candidate_pose;
        candidates_.push_back(candidate);

        // Draw particle visualization
        visualization::DrawParticle(candidate_pose.loc, candidate_pose.angle, vis_msg_);
      }
    }
  }

  // Publish particles visualization
  vis_msg_.header.stamp = ros::Time::now();
  vis_pub_.publish(vis_msg_);

  // Set motion model flag
  motion_model_ready_ = true;
}

vector<Vector2f> SLAM::GetMap() {
  vector<Vector2f> map;
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.
  return map;
}


void SLAM::StateTransform(int ind2, int ind1){
  State statey;
  Eigen::Vector2f vec(1.0f, 1.0f);
  statey.pose.loc = vec;
  statey.pose.angle = 0;
  state_chain_.push_back(statey);
  state_chain_.push_back(statey);
  state_chain_.push_back(statey);
  state_chain_.push_back(statey);

  if (static_cast<int> (state_chain_.size()) <= ind2+1) return;

  // Accumulate the transforms between the two states
  Eigen::Matrix3f transform;
  transform << cos(cos(state_chain_[ind1+1].pose.angle)), -sin(state_chain_[ind1+1].pose.angle), state_chain_[ind1+1].pose.loc.x(),
                sin((state_chain_[ind1+1].pose.angle)), cos(state_chain_[ind1+1].pose.angle), state_chain_[ind1+1].pose.loc.y(),
                0, 0, 1;
  if (ind2 - ind1 > 1){
    for (int i = ind1+2; i <= ind2; i++){
      Eigen::Matrix3f chain_transform;
      chain_transform << cos(cos(state_chain_[i].pose.angle)), -sin(state_chain_[i].pose.angle), state_chain_[i].pose.loc.x(),
                        sin((state_chain_[i].pose.angle)), cos(state_chain_[i].pose.angle), state_chain_[i].pose.loc.y(),
                        0, 0, 1;
      transform = transform*chain_transform;
    }
  }
  
  std::cout << transform << std::endl;

}


}  // namespace slam
