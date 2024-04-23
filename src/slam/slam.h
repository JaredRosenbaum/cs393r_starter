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
#include <variant>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "ros/ros.h"
#include "ros/package.h"
#include "visualization/visualization.h"
#include "amrl_msgs/VisualizationMsg.h"

#include "rasterization.hpp"
#include "motion_model.hpp"
#include "parameters.h"
#include "icp.h"

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
    Pose rel_odom;
    
    int graph_id;
    gtsam::Pose2 abs_pose;
    gtsam::Pose2 rel_pose;
    gtsam::Matrix rel_cov;
}; // struct NonsequentialNode

struct SequentialNode {
    const int id;
    const Pose rel_odom;
    const std::vector<Eigen::Vector2f> points; 
    std::shared_ptr<rasterization::LookupTable> lookup_table;
    
    std::vector<NonsequentialNode> nodes;
    
    int graph_id;
    gtsam::Pose2 abs_pose;
    gtsam::Pose2 rel_pose;
    gtsam::Matrix rel_cov;

    gtsam::Pose2 est_pose;

    SequentialNode(const int &identifier, const Pose &odom, const std::vector<Eigen::Vector2f> &cloud) 
    : id(identifier), rel_odom(odom), points(cloud)
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

  // Observe new odometry-reported location.
  void ObserveOdometry(const Eigen::Vector2f& odom_loc,
                       const float odom_angle);

  // Get latest map.
  std::vector<Eigen::Vector2f> GetMap();

  // Get latest robot pose.
  void GetPose(Eigen::Vector2f* loc, float* angle);

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
  int gtsam_timer_;
  std::vector<std::shared_ptr<SequentialNode>> chain_;

  gtsam::Pose2 starting_pose_;
  
  void optimizeChain();

  void iterateSLAM(
    Pose &odom,
    std::vector<Eigen::Vector2f> &cloud);
  
  std::variant<bool, std::pair<gtsam::Pose2, gtsam::Matrix>> pairwiseComparison(
      std::shared_ptr<SequentialNode> &new_node,
      std::shared_ptr<SequentialNode> &existing_node);

  std::shared_ptr<std::vector<Candidate>> generateCandidates(
      Pose odom,
      std::vector<Eigen::Vector2f> points,
      std::shared_ptr<rasterization::LookupTable> &ref,
      const std::vector<Eigen::Vector2f> &ref_points);
  
  std::variant<bool, Eigen::Matrix3d> calculateCovariance(std::shared_ptr<std::vector<Candidate>> &candidates);

  void detectLoops(std::shared_ptr<SequentialNode> &state);
  double computeDisplacement(const SequentialNode &state1, const SequentialNode &state2);

  Eigen::Matrix3f getTransformChain(int ind2, int ind1=0);
  Eigen::Matrix3f pose2Transform(const Pose &pose);

  void transformPose(Pose &pose, const Eigen::Matrix3f &T);
  void transformPose(Pose &pose, const Pose &P);
  Pose transformPoseCopy(const Pose &pose, const Eigen::Matrix3f &T);
  Pose transformPoseCopy(const Pose &pose, const Pose &P);
  gtsam::Pose2 transformPoseCopy(const gtsam::Pose2 &pose, const Eigen::Matrix3f &T);
  gtsam::Pose2 transformPoseCopy(const gtsam::Pose2 &pose, const gtsam::Pose2 &P);
  void transformPoses(std::vector<Pose> &poses, const Eigen::Matrix3f &T);
  void transformPoses(std::vector<Pose> &poses, const Pose &pose);
  void transformPoint(Eigen::Vector2f &point, const Eigen::Matrix3f &T);
  void transformPoint(Eigen::Vector2f &point, const Pose &pose);
  void transformPoints(std::vector<Eigen::Vector2f> &points, const Eigen::Matrix3f &T);
  void transformPoints(std::vector<Eigen::Vector2f> &points, const Pose &pose);

}; // class SLAM

}  // namespace slam

#endif   // SRC_SLAM_H_
