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
    reference_point_cloud_.clear();
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

      // Convert to map frame
      const Eigen::Rotation2Df rotation_transform(reference_scan_pose_.angle);
      point = reference_scan_pose_.loc + rotation_transform * point;

      // Push into point cloud vector
      reference_point_cloud_.push_back(point);

      // Draw point visualization
      visualization::DrawPoint(point, 0x5de053, vis_msg_);
    }

    // Publish point cloud visualization
    vis_msg_.header.stamp = ros::Time::now();
    vis_pub_.publish(vis_msg_);

    // Proceed with transforming laser scan into every candidate of the motion model
    PrepareLaserTransformations(reference_point_cloud_);
  }
}

void SLAM::PrepareLaserTransformations(const std::vector<Eigen::Vector2f> &point_cloud) {
  // Loop through motion model candidate poses aligning the point cloud from the reference
  for (auto &candidate : candidates_) {

    // Calculate transformation from candidate pose to reference
    // Eigen::Vector2f translation = candidate.pose.loc - reference_scan_pose_.loc;
    Eigen::Rotation2Df rotation_transform(AngleDiff(candidate.pose.angle, reference_scan_pose_.angle));

    // TODO Optimize for not recalculating pointcloud here?

    // Loop through reference pointcloud transforming it to candidate frame
    for (const auto &point : point_cloud) {
      Eigen::Vector2f new_point = rotation_transform * (point - reference_scan_pose_.loc) + candidate.pose.loc;

      visualization::DrawPoint(new_point, 0x536de0, vis_msg_);
    }

    // Publish point cloud visualization
    vis_msg_.header.stamp = ros::Time::now();
    vis_pub_.publish(vis_msg_);
    std::cout << "AAAA";
    std::cin.get();

    visualization::ClearVisualizationMsg(vis_msg_);
  }

  // Clear motion model flag
  motion_model_ready_ = false;
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
      Pose odom_change;
      odom_change.loc = odom_loc - reference_odom_pose_.loc;
      odom_change.angle = AngleDiff(odom_angle, reference_odom_pose_.angle);

      // std::cout << odom_change.loc.x() << "\t" << odom_change.loc.y() << "\t" << odom_change.angle << "\t" << std::endl;

      float theta = atan2(odom_change.loc.y(), odom_change.loc.x());
      float x = odom_change.loc.norm() * sin(theta);
      float y = odom_change.loc.norm() * cos(theta);

      std::cout << x << "\t" << y << "\t" << theta << std::endl;


      // Proceed with preparing a motion model when meeting threshold
      if (odom_change.loc.norm() > ODOM_TRANSLATION_THRESHOLD ||
        std::abs(odom_change.angle) > ODOM_ROTATION_THRESHOLD) {
        // // Transform odometry change into the car frame
        // Eigen::Rotation2Df rotation_transform(reference_odom_pose_.angle);
        // Pose state_pose_change;
        // state_pose_change.loc = rotation_transform * odom_change.loc;
        // state_pose_change.angle = odom_change.angle;

        // PrepareMotionModel(state_pose_change);
      }

      // // Transform odometry pose change to map frame
      // Eigen::Rotation2Df rotation_transform(AngleDiff(current_pose_.angle, odom_angle));
      // Eigen::Vector2f map_translation = rotation_transform * odom_translation;

      // // Keep track of pose change from odometry to estimate how far the robot has moved. Update robot location (in map frame) based on this
      // current_pose_.loc += map_translation;
      // current_pose_.angle += odom_rotation;

      // // Calculate pose change from previous successful scan match
      // Eigen::Vector2f scan_match_translation = current_pose_.loc - reference_scan_pose_.loc;
      // float scan_match_rotation = current_pose_.angle - reference_scan_pose_.angle;

      // // Proceed with preparing a motion model when meeting threshold
      // if (scan_match_translation.norm() > ODOM_TRANSLATION_THRESHOLD || scan_match_rotation > ODOM_ROTATION_THRESHOLD) {
      //   PrepareMotionModel(current_pose_);
      // }

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




  // // Clear motion model and particle visualization
  // candidates_.clear();
  // visualization::ClearVisualizationMsg(vis_msg_);

  // // Calculate number of divisions (1/2 loop iterations) in each dimension x, y, theta
  // const static int x_iterations = std::ceil(MOTION_MODEL_X_LIMIT / MOTION_MODEL_X_RESOLUTION);
  // const static int y_iterations = std::ceil(MOTION_MODEL_Y_LIMIT / MOTION_MODEL_Y_RESOLUTION);
  // const static int theta_iterations = std::ceil(MOTION_MODEL_THETA_LIMIT / MOTION_MODEL_THETA_RESOLUTION);

  // // Triple loop to populate motion model poses with variance in each dimension. This is esssentially a cube of possibilities
  // for (int theta_i = -theta_iterations; theta_i <= theta_iterations; theta_i++) {
  //   for (int  y_i = -y_iterations; y_i <= y_iterations; y_i++) {
  //     for (int x_i = -x_iterations; x_i <= x_iterations; x_i++) {
  //       // Create pose candidate in map frame. Rotate volume based on direction of car
  //       const Eigen::Rotation2Df rotation_transform(pose_in_map.angle);
  //       Pose candidate_pose;
  //       candidate_pose.loc = pose_in_map.loc + rotation_transform * Eigen::Vector2f{
  //         x_i * MOTION_MODEL_X_RESOLUTION,
  //         y_i * MOTION_MODEL_Y_RESOLUTION
  //       };
  //       candidate_pose.angle = pose_in_map.angle + theta_i * MOTION_MODEL_THETA_RESOLUTION;

  //       // Push a new candidate into the candidates vector
  //       Candidate candidate;
  //       candidate.pose = candidate_pose;
  //       candidates_.push_back(candidate);

  //       // Draw particle visualization
  //       visualization::DrawParticle(candidate_pose.loc, candidate_pose.angle, vis_msg_);
  //     }
  //   }
  // }

  // // Publish particles visualization
  // vis_msg_.header.stamp = ros::Time::now();
  // vis_pub_.publish(vis_msg_);

  // // Update reference scan pose and set motion model flag
  // reference_scan_pose_ = pose_in_map;
  // motion_model_ready_ = true;
}

vector<Vector2f> SLAM::GetMap() {
  vector<Vector2f> map;
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.
  return map;
}

}  // namespace slam
