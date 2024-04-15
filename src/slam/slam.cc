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
    odom_initialized_(false) {}

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
    // Loop through motion model candidate poses aliging received laser scan
    for (auto &candidate : motion_model_) {
      // Calculate transformation from candidate pose to reference
      Eigen::Vector2f translation = candidate.loc - reference_scan_pose_.loc;
      float rotation = AngleDiff(candidate.angle, candidate.angle);

      // Transform laser scan to candidate pose frame
      // TODO scan comes from "base_laser" should this be adjusted to "base_link" first?

    }


    motion_model_ready_ = false;  // TODO Can this be moved out? Do ROS nodes have race conditions?
  }
}

void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
  // Keep track of odometry to estimate how far the robot has moved between poses.



  // std::cout << odom_loc.x() << "\t" << odom_loc.y() << "\t" << odom_angle << std::endl;

  // Disregard first odometry reading, set pose variables
  if (!odom_initialized_) {
    // current_pose_.loc = Vector2f(0, 0);
    // current_pose_.angle = 0;

    // reference_scan_pose_.loc = Vector2f(0,0);
    // reference_scan_pose_.angle = 0;

    reference_scan_pose_.loc = current_pose_.loc;
    reference_scan_pose_.angle = current_pose_.angle;

    odom_initialized_ = true;
  }

  else {
    // Calculate pose change from odometry reading
    Eigen::Vector2f odom_translation = odom_loc - prev_odom_pose_.loc; // TODO Moving forward gives a negative difference for some reason
    float odom_rotation = AngleDiff(odom_angle, prev_odom_pose_.angle);

    // Ignore unrealistic jumps in odometry
    if (odom_translation.norm() < 1.0 && abs(odom_rotation) < M_PI_4) {
      // Transform odometry pose change to map frame
      Eigen::Rotation2Df rotation_transform(AngleDiff(current_pose_.angle, odom_angle));
      Eigen::Vector2f map_translation = rotation_transform * odom_translation;

      // Update robot location based purely on odometry
      current_pose_.loc += map_translation;
      current_pose_.angle += odom_rotation;

      // Calculate pose change from previous successful scan match
      Eigen::Vector2f scan_match_translation = current_pose_.loc - reference_scan_pose_.loc;
      float scan_match_rotation = current_pose_.angle - reference_scan_pose_.angle;

      // Proceed with preparing a motion model when meeting threshold
      if (scan_match_translation.norm() > ODOM_TRANSLATION_THRESHOLD || scan_match_rotation > ODOM_ROTATION_THRESHOLD) {
        PrepareMotionModel(current_pose_);
      }
    }
  }

  // Update previous odometry
  prev_odom_pose_.loc = odom_loc;
  prev_odom_pose_.angle = odom_angle;
}

void SLAM::PrepareMotionModel(const Pose pose) {
  // Clear particle visualization
  visualization::ClearVisualizationMsg(vis_msg_);

  // Calculate number of divisions (1/2 loop iterations) in each dimension x, y, theta
  const static int x_iterations = std::ceil(MOTION_MODEL_X_LIMIT / MOTION_MODEL_X_RESOLUTION);
  const static int y_iterations = std::ceil(MOTION_MODEL_Y_LIMIT / MOTION_MODEL_Y_RESOLUTION);
  const static int theta_iterations = std::ceil(MOTION_MODEL_THETA_LIMIT / MOTION_MODEL_THETA_RESOLUTION);

  // std::cout << pose.loc.x() << "\t" << pose.loc.y() << "\t" << pose.angle << std::endl;
  // std::cout << "=====" << std::endl;

  // int x_iterations = std::ceil(MOTION_MODEL_X_LIMIT / MOTION_MODEL_X_RESOLUTION);
  // int y_iterations = std::ceil(MOTION_MODEL_X_LIMIT / MOTION_MODEL_X_RESOLUTION);
  // int theta_iterations = std::ceil(MOTION_MODEL_X_LIMIT / MOTION_MODEL_X_RESOLUTION);

  // Triple loop to populate motion model poses with variance in each dimension. This is esssentially a cube of possibilities
  for (int theta_i = -theta_iterations; theta_i <= theta_iterations; theta_i++) {
    for (int  y_i = -y_iterations; y_i <= y_iterations; y_i++) {
      for (int x_i = -x_iterations; x_i <= x_iterations; x_i++) {
        // Create pose candidate
        Pose candidate; // TODO Add constructor?
        candidate.loc = Eigen::Vector2f{
          pose.loc.x() + x_i * MOTION_MODEL_X_RESOLUTION,
          pose.loc.y() + y_i * MOTION_MODEL_Y_RESOLUTION,
        };
        candidate.angle = pose.angle + theta_i * MOTION_MODEL_THETA_RESOLUTION;

        // std::cout << x_i << "\t";

        // std::cout << candidate.loc.x() << "\t" << candidate.loc.y() << "\t" << candidate.angle << std::endl;

        // Draw particle visualization
        visualization::DrawParticle(candidate.loc, candidate.angle, vis_msg_);
      }
      // std::cout << "-----" << std::endl;
    }
  }

  // Publish particle visualization
  vis_msg_.header.stamp = ros::Time::now();
  vis_pub_.publish(vis_msg_);

  // std::cin.get();

  // Update reference scan pose and set motion model flag
  reference_scan_pose_ = current_pose_;
  motion_model_ready_ = true;
}

vector<Vector2f> SLAM::GetMap() {
  vector<Vector2f> map;
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.
  return map;
}

}  // namespace slam
