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
\file    slam.h
\brief   SLAM Interface
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

#include <algorithm>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "ros/ros.h"
#include "ros/package.h"
#include "visualization/visualization.h"
#include "amrl_msgs/VisualizationMsg.h"

#ifndef SRC_SLAM_H_
#define SRC_SLAM_H_

namespace slam {

struct Pose {
  Eigen::Vector2f loc;
  float angle;
};

struct Candidate {
  Pose pose;    // relative to previous pose frame
  // std::vector<Eigen::Vector2f> point_cloud;
  double p_motion = 0;
  double p_scan = 0;
};

struct State {
  Pose pose;      // relative to previous pose frame
  Eigen::Matrix3f cov;
  std::vector<Eigen::Vector2f> point_cloud;   // relative to current pose frame
}

class SLAM {
 public:
  // Default Constructor.
  SLAM();

  // Create visualization publisher
  void CreateVisPublisher(ros::NodeHandle* n);

  // Initialize pose to align with web visualizer
  void InitializePose(const Eigen::Vector2f& loc, const float angle);

  // Observe a new laser scan.
  void ObserveLaser(const std::vector<float>& ranges,
                    float range_min,
                    float range_max,
                    float angle_min,
                    float angle_max);
  
  // Compute laser scan transformations for motion model
  void PrepareLaserTransformations(const std::vector<Eigen::Vector2f> &point_cloud);

  // Observe new odometry-reported location.
  void ObserveOdometry(const Eigen::Vector2f& odom_loc,
                       const float odom_angle);
  
  // Calculate motion model for scan matching
  void PrepareMotionModel(const Pose pose_in_map);

  // Get latest map.
  std::vector<Eigen::Vector2f> GetMap();

  // Get latest robot pose.
  void GetPose(Eigen::Vector2f* loc, float* angle) const;

 private:
  ros::Publisher vis_pub_;
  amrl_msgs::VisualizationMsg vis_msg_;

  Pose current_pose_;     // current estimate of the robot pose
  Pose reference_scan_pose_;  // reference scan match pose

  std::vector<Candidate> candidates_;   // candidate vector for CSM algorithm

  Pose prev_odom_pose_;   // previous odometry-reported pose
  bool odom_initialized_; // odometry flag
  std::vector<Pose> motion_model_;  // motion model vector of poses
  bool motion_model_ready_;    // motion model 
  
  std::vector<Eigen::Vector2f> reference_point_cloud_;  // reference laser scan

  std::vector<State> state_chain_;

};
}  // namespace slam

#endif   // SRC_SLAM_H_
