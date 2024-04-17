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
#include <memory>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "ros/ros.h"
#include "ros/package.h"
#include "visualization/visualization.h"
#include "amrl_msgs/VisualizationMsg.h"

#include "rasterization.hpp"
#include "motion_model.hpp"
#include "parameters.h"

#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <boost/optional.hpp>

#ifndef SRC_SLAM_H_
#define SRC_SLAM_H_

namespace slam {

struct Pose {
    Eigen::Vector2f loc;
    float angle;
    Pose() {}
    Pose (float x, float y, float theta)
    : loc(Eigen::Vector2f(x, y)), angle(theta) {}
}; // struct Pose

struct Candidate {
    Pose relative_pose;
    double p_motion {};
    double p_scan {};
    double p {};
}; // struct Candidate

struct NonsequentialNode {
    int parent;
    int graph_id;
    // int child;
    Pose relative_pose;
    Eigen::Matrix3d relative_covariance;
}; // struct NonsequentialNode

struct SequentialNode {
    const int id;
    int graph_id;
    // int child;
    Pose relative_pose;
    Eigen::Matrix3d relative_covariance;

    const Pose raw_odometry;
    const std::vector<Eigen::Vector2f> points; 
    std::shared_ptr<rasterization::LookupTable> lookup_table;
    std::vector<NonsequentialNode> nodes;

    SequentialNode(const int &identifier, const Pose &odom, const std::vector<Eigen::Vector2f> &cloud) 
    : id(identifier), raw_odometry(odom), points(cloud)
    {
        // raw_odometry = odom;
        lookup_table = std::make_shared<rasterization::LookupTable>(points, LOOKUP_TABLE_RESOLUTION, LOOKUP_TABLE_SIGMA);
    }
}; // struct SequentialNode

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
  
  // Compute point clouds for each candidate and score their probabilities
  void ConfigureCandidates(const std::vector<Eigen::Vector2f> &point_cloud, const std::vector<Eigen::Vector2f> &stored_point_cloud);

  // Observe new odometry-reported location.
  void ObserveOdometry(const Eigen::Vector2f& odom_loc,
                       const float odom_angle);
  
  // Calculate motion model for scan matching
  void PrepareMotionModel(const Pose odom_change);

  // Get latest map.
  std::vector<Eigen::Vector2f> GetMap();

  // Get latest robot pose.
  void GetPose(Eigen::Vector2f* loc, float* angle) const;

 private:
  ros::Publisher vis_pub_;
  amrl_msgs::VisualizationMsg vis_msg_;

  bool odom_initialized_; // odometry flag
  Pose prev_odom_pose_;   // previous odometry-reported pose
  Pose reference_odom_pose_;
  Pose odom_change_;
  bool ready_for_slam_update_;    // motion model 
  Pose current_pose_;

  // !!!
  int depth_;
  std::vector<std::shared_ptr<SequentialNode>> chain_;

  void optimizeChain();

  void update(
    Pose &odom,
    std::vector<Eigen::Vector2f> &cloud);

  void updatePairwiseSequential(
        std::shared_ptr<SequentialNode> &new_node,
        std::shared_ptr<SequentialNode> &existing_node);
    
  void updatePairwiseNonsequential(
      std::shared_ptr<SequentialNode> &new_node,
      std::shared_ptr<SequentialNode> &existing_node);
  
  std::pair<Pose, Eigen::Matrix3d> coreUpdate(
      std::shared_ptr<SequentialNode> &new_node,
      std::shared_ptr<SequentialNode> &existing_node);

  std::shared_ptr<std::vector<Candidate>> generateCandidates(
      Pose odom,
      std::vector<Eigen::Vector2f> points,
      std::shared_ptr<rasterization::LookupTable> &ref,
      const std::vector<Eigen::Vector2f> &ref_points);
  
  Eigen::Matrix3d calculateCovariance(std::shared_ptr<std::vector<Candidate>> &candidates);

  Eigen::Matrix3f getTransformChain(int ind2, int ind1=0);
  Eigen::Matrix3f pose2Transform(const Pose &pose);
  void transformPoses(std::vector<Pose> &poses, const Eigen::Matrix3f &T);
  void transformPoses(std::vector<Pose> &poses, const Pose &pose);
  void transformPoints(std::vector<Eigen::Vector2f> &points, const Eigen::Matrix3f &T);
  void transformPoints(std::vector<Eigen::Vector2f> &points, const Pose &pose);
  void transformPose(Pose &pose, const Eigen::Matrix3f &T);
  void transformPose(Pose &p, const Pose &pose);
  void transformPoint(Eigen::Vector2f &point, const Eigen::Matrix3f &T);
  void transformPoint(Eigen::Vector2f &point, const Pose &pose);

}; // class SLAM

}  // namespace slam

#endif   // SRC_SLAM_H_
