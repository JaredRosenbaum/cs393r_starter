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
}

void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {

  // std::cout << odom_loc.x() << "\t" << odom_loc.y() << "\t" << odom_angle << std::endl;

  // Disregard first odometry reading and reset variables
  if (!odom_initialized_) {
    current_pose_.loc = Vector2f(0, 0);
    current_pose_.angle = 0;

    reference_scan_pose_.loc = Vector2f(0,0);
    reference_scan_pose_.angle = 0;

    odom_initialized_ = true;
  }

  else {
    // Keep track of odometry to estimate how far the robot has moved between poses.

    // Calculate pose change from odometry reading
    Eigen::Vector2f odom_translation = odom_loc - prev_odom_pose_.loc; // TODO Moving forward gives a negative difference for some reason
    float odom_rotation = AngleDiff(odom_angle, prev_odom_pose_.angle);

    // Ignore unrealistic jumps in odometry
    if (odom_translation.norm() < 1.0 && abs(odom_rotation) < M_PI_4) {
      // Update robot location based purely on odometry
      current_pose_.loc += odom_translation;
      current_pose_.angle += odom_rotation;

      // std::cout << translation_diff.x() << "\t" << translation_diff.y() << "\t" << rotation_diff << std::endl;
      // std::cout << current_pose_loc_.x() << "\t" << current_pose_loc_.y() << "\t" << current_pose_angle_ << std::endl;
      // std::cout << "---" << std::endl;

      // Calculate pose change from previous successful scan match
      Eigen::Vector2f scan_match_translation = current_pose_.loc - reference_scan_pose_.loc;
      float scan_match_rotation = current_pose_.angle - reference_scan_pose_.angle;

      // Proceed with preparing a motion model when meeting threshold
      if (scan_match_translation.norm() > ODOM_TRANSLATION_THRESHOLD || scan_match_rotation > ODOM_ROTATION_THRESHOLD) {
        PrepareMotionModel(scan_match_translation, scan_match_rotation);
      }
    }
  }

  // Update previous odometry
  prev_odom_pose_.loc = odom_loc;
  prev_odom_pose_.angle = odom_angle;
}

void SLAM::PrepareMotionModel(const Eigen::Vector2f translation, const float rotation) {
  


  motion_model_ready = true;
}

vector<Vector2f> SLAM::GetMap() {
  vector<Vector2f> map;
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.
  return map;
}

}  // namespace slam
