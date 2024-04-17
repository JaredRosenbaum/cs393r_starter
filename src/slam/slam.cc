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

// TODO
// + make SLAM start from current robot location instead of origin (for testing and visualization, plus should get rid of robot jumping around)
// + fill in GetMap function
// + make version of chain transform function that uses odom instead of relative poses, this should be used for relative odometry between non-sequential poses
// + test with different parameters for scan matching and motion model (figure out why scan drifts over time and how to fix it)
// + fix visualization of selected cloud and pose in coreUpdate function for debugging
// + finish GTSAM implementation and test
// + covariances are sometimes NaN, figure out when this happens and how to fix it
// + consider changing sampling limits and resolutions relative to odometry (so if we've moved farther, increase the size of the search space but keep the same number of samples?)

#include <algorithm>
#include <cmath>
#include <iostream>
#include <chrono>
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
    ready_for_slam_update_(false),
    depth_(POSE_GRAPH_CONNECTION_DEPTH) {}

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
  if (!ready_for_slam_update_) {
    return;
  }

  std::vector<Eigen::Vector2f> point_cloud;

  // Convert laser scan to point cloud centered around base_link
  float angle_inc = (angle_max - angle_min) / ranges.size();
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
    point_cloud.push_back(point);

    // Draw point visualization
    // visualization::DrawPoint(point, 0x5de053, vis_msg_);
  }

  // Publish point cloud visualization
  vis_msg_.header.stamp = ros::Time::now();
  vis_pub_.publish(vis_msg_);

  // update SLAM
  update(odom_change_, point_cloud);

  // Clear motion model flag
  ready_for_slam_update_ = false;
}

void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {

  // std::cout << odom_loc.x() << "\t" << odom_loc.y() << "\t" << odom_angle << std::endl;

  // Disregard first odometry reading, set pose variables
  if (!odom_initialized_) {
    reference_odom_pose_.loc = odom_loc;
    reference_odom_pose_.angle = odom_angle;

    odom_initialized_ = true;
    return;
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
      // std::cout << translation << ", " << rotation << std::endl;

      // Proceed with preparing a motion model when meeting threshold
      if ((translation > ODOM_TRANSLATION_THRESHOLD ||
        std::abs(rotation) > ODOM_ROTATION_THRESHOLD) && !ready_for_slam_update_) {
        // Calculate the change in odometry in reference to the odom frame (reference odom pose is (0, 0, 0))
        odom_change_.loc = Eigen::Vector2f(
          translation * cos(rotation),
          translation * sin(rotation)
        );
        odom_change_.angle = rotation;
        ready_for_slam_update_ = true;

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

vector<Vector2f> SLAM::GetMap() {
  vector<Vector2f> map;
    // TODO, should be straightforward using the same process as for visualization
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.
  return map;
}

// !!! SLAM STUFF

// -------------------------------------------
// * Primary Update
// -------------------------------------------

void SLAM::update(
    Pose &odom,
    std::vector<Eigen::Vector2f> &cloud)
{
    auto start_time {std::chrono::high_resolution_clock::now()};
    std::cout << "\n\n=====================================" << std::endl;
    std::cout << "UPDATE" << std::endl;
    std::cout << "=====================================" << std::endl;

    // . add new sequential state
    std::cout << "Creating new state with " << cloud.size() << " points." << std::endl;
    auto new_state {std::make_shared<SequentialNode>(chain_.size(), odom, cloud)};
    // new_state->lookup_table->exportAsPPM("/home/dev/cs393r_starter/images/test.ppm"); // ! this may be useful for debugging but it's like 10x slower to do

    // compare with previous state and populate pose and covariance
    if (!chain_.empty()) {
        updatePairwiseSequential(new_state, chain_[chain_.size() - 1]);
    } else {
        new_state->relative_pose = Pose(0, 0, 0);
        new_state->relative_covariance = Eigen::Matrix3d::Zero();
    }

    std::cout << "Pose:\n" << new_state->relative_pose.loc.x() << ", " << new_state->relative_pose.loc.y() << ", " << new_state->relative_pose.angle << "\nCov:\n" << new_state->relative_covariance << std::endl;

    // and add it to the chain
    chain_.push_back(new_state);
    std::cout << "State " << new_state->id << " added to chain!" << std::endl;

    // . now create all non-sequential states
    // for each comparison
    for (int i = 0; i < depth_; i++) {
        int index {(static_cast<int>(chain_.size()) - 1) - i - 1};
        if (index < 0) {continue;}
        std::cout << "\nAdding nonsequential relation with " << index << std::endl;
        updatePairwiseNonsequential(new_state, chain_[index]);
    }

    // . perform pose graph optimization using GTSAM
    optimizeChain();

    // . now add visualization stuff here for debugging
    // clear visualization
    visualization::ClearVisualizationMsg(vis_msg_);

    std::cout << "Trying to visualize..." << std::endl;

    for (int i = 0; i < static_cast<int>(chain_.size()); i++) {
        // get transform to root of chain
        // std::cout << i << std::endl;
        Eigen::Matrix3f root_tf {getTransformChain(i)};

        // std::cout << "Transform between root and " << i << ":\n" << root_tf << std::endl;

        // transform cloud
        auto viz_points {chain_[i]->points};
        transformPoints(viz_points, root_tf);

        // transform pose
        auto viz_pose {chain_[i]->relative_pose};
        transformPose(viz_pose, root_tf);

        // visualize cloud
        for (const auto &point : viz_points) {
            visualization::DrawPoint(point, 0xF633FF, vis_msg_);
        }

        // visualize pose
        visualization::DrawParticle(viz_pose.loc, viz_pose.angle, vis_msg_);

        vis_msg_.header.stamp = ros::Time::now();
        vis_pub_.publish(vis_msg_);

        // std::string input;
        // std::getline(std::cin, input);

        // maybe draw line between poses? doesn't seem necessary
    }

    auto end_time {std::chrono::high_resolution_clock::now()};
    auto time_duration {std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time)};
    std::cout << ">> Finished SLAM update in " << time_duration.count() << "us" << std::endl;
}

// -------------------------------------------
// * Pose Graph Optimization
// -------------------------------------------

void SLAM::optimizeChain()
{
    // TODO iterate over chain and add poses and covariances as factors, then give initial estimate and optimize, then update chain using the results

    // create GTSAM graph
    gtsam::NonlinearFactorGraph graph;

    // iterate over chain and add all points and initial estimates
    // ? need to associate poses with unique indexes for recovery
    int graph_id {0};

    // start by adding sequential poses to graph, make sure they don't change with optimization 
    for (const auto &node : chain_) {
        auto relative_pose {gtsam::Pose2(node->relative_pose.loc.x(), node->relative_pose.loc.y(), node->relative_pose.angle)};

        gtsam::Matrix relative_covariance {node->relative_covariance};
        graph.add(gtsam::BetweenFactor<gtsam::Pose2>(graph_id, graph_id + 1, relative_pose, gtsam::noiseModel::Gaussian::Covariance(relative_covariance)));

        graph_id++;
    }

    // optimize the graph
    graph.print();

    // use results to update the stored chain


}

// -------------------------------------------
// * Updating Pairs
// -------------------------------------------

void SLAM::updatePairwiseSequential(
    std::shared_ptr<SequentialNode> &new_node,
    std::shared_ptr<SequentialNode> &existing_node)
{
    auto start_time {std::chrono::high_resolution_clock::now()};

    // perform the update (results same for sequenial and non-sequential scans, but the way they are stored is different)
    auto results {coreUpdate(new_node, existing_node)};
    new_node->relative_pose = results.first;
    new_node->relative_covariance = results.second;

    auto end_time {std::chrono::high_resolution_clock::now()};
    auto time_duration {std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time)};
    std::cout << ">> Finished sequential pose update in " << time_duration.count() << "us" << std::endl;
}

void SLAM::updatePairwiseNonsequential(
    std::shared_ptr<SequentialNode> &new_node,
    std::shared_ptr<SequentialNode> &existing_node)
{
    // perform the update (results same for sequenial and non-sequential scans, but the way they are stored is different)
    auto results {coreUpdate(new_node, existing_node)};

    NonsequentialNode node;
    node.parent = existing_node->id;
    node.relative_pose = results.first;
    node.relative_covariance = results.second;
    new_node->nodes.push_back(node);

    std::cout << "Pose:\n" << node.relative_pose.loc.x() << ", " << node.relative_pose.loc.y() << ", " << node.relative_pose.angle << "\nCov:\n" << node.relative_covariance << std::endl;
}

std::pair<Pose, Eigen::Matrix3d> SLAM::coreUpdate(
    std::shared_ptr<SequentialNode> &new_node,
    std::shared_ptr<SequentialNode> &existing_node)
{
    // generate candidates
    auto candidates {generateCandidates(new_node->raw_odometry, new_node->points, existing_node->lookup_table, existing_node->points)};

    // TODO make sure transforms are generated correctly, see which term tends to impact p more
    // for (const auto &candidate : *candidates) {
    //     std::cout << "\t" << candidate.p_motion << ", " << candidate.p_scan << ", " << candidate.p << std::endl;
    // }

    // calculate covariance
    auto covariance {calculateCovariance(candidates)};

    // and get the best pose
    Pose best_pose {(*candidates)[0].relative_pose};
    double best_prob {(*candidates)[0].p};
    for (std::size_t i = 1; i < candidates->size(); i++) {
        if ((*candidates)[i].p > best_prob) {
            best_prob = (*candidates)[i].p;
            best_pose = (*candidates)[i].relative_pose;
        }
    }

    // . visualize scan and pose compared to reference (this doesn't exactly work right now, need to test it out more)
    // visualization::ClearVisualizationMsg(vis_msg_);

    // // draw reference cloud
    // for (const auto &point : existing_node->points) {
    //     visualization::DrawPoint(point, 0xF633FF, vis_msg_);
    // }

    // // draw projected cloud
    // auto cloud {new_node->points};
    // Eigen::Matrix3f transform {pose2Transform(best_pose)};
    // // std::cout << "\tProduced cloud transform:\n" << transform << std::endl;
    // transformPoints(cloud, transform);
    // for (const auto &point : cloud) {
    //     visualization::DrawPoint(point, 0x33FF50, vis_msg_);
    // }

    // // draw pose
    // visualization::DrawParticle(best_pose.loc, best_pose.angle, vis_msg_);

    // // publish msg
    // vis_msg_.header.stamp = ros::Time::now();
    // vis_pub_.publish(vis_msg_);

    // // wait for input
    // std::string input;
    // std::getline(std::cin, input);

    // clear visualization
    visualization::ClearVisualizationMsg(vis_msg_);

    return std::make_pair<Pose, Eigen::Matrix3d>(std::move(best_pose), std::move(covariance));
}

// -------------------------------------------
// * Generating Candidates
// -------------------------------------------

std::shared_ptr<std::vector<Candidate>> SLAM::generateCandidates(
    Pose odom,
    std::vector<Eigen::Vector2f> points,
    std::shared_ptr<rasterization::LookupTable> &ref,
    const std::vector<Eigen::Vector2f> &ref_points)
{
    // std::shared_ptr<std::vector<Candidate>> candidates;
    auto candidates {std::make_shared<std::vector<Candidate>>()};

    // create motion model
    auto motion_model {motion_model::MultivariateMotionModel(Eigen::Vector3d(odom.loc.x(), odom.loc.y(), odom.angle))};

    // iterate over candidates
    const int x_iterations = std::ceil(MOTION_MODEL_X_LIMIT / MOTION_MODEL_X_RESOLUTION);
    const int y_iterations = std::ceil(MOTION_MODEL_Y_LIMIT / MOTION_MODEL_Y_RESOLUTION);
    const int theta_iterations = std::ceil(MOTION_MODEL_THETA_LIMIT / MOTION_MODEL_THETA_RESOLUTION);

    // reserving space in candidates
    candidates->reserve(x_iterations * y_iterations * theta_iterations);

    // Triple loop to populate motion model poses with variance in each dimension. This creates a cube of next state possibilities
    for (int theta_i = -theta_iterations; theta_i <= theta_iterations; theta_i++) {

        Candidate candidate;
        // ! rotate the point cloud (expensive!)
        // !!! realize we will need to do the combined transform for both the odom and this block
        // auto cloud {points}; 
        // transformPoints(cloud, Pose(0, 0, theta_i + odom.angle));

        for (int  y_i = -y_iterations; y_i <= y_iterations; y_i++) {
            for (int x_i = -x_iterations; x_i <= x_iterations; x_i++) {
                
                // . generate a candidate pose WRT to the body
                auto candidate_pose {Pose(
                        static_cast<float>(x_i * MOTION_MODEL_X_RESOLUTION), 
                        static_cast<float>(y_i * MOTION_MODEL_Y_RESOLUTION), 
                        static_cast<float>(theta_i * MOTION_MODEL_THETA_RESOLUTION))
                };
                // std::cout << "Odom: " << odom.loc.x() << ", " << odom.loc.y() << ", " << odom.angle << std::endl;
                // std::cout << "Candidate: " << candidate_pose.loc.x() << ", " << candidate_pose.loc.y() << ", " << candidate_pose.angle << std::endl;

                // . transform it back to the last frame (from odom)
                transformPose(candidate_pose, odom);
                // std::cout << "Transformed: " << candidate_pose.loc.x() << ", " << candidate_pose.loc.y() << ", " << candidate_pose.angle << std::endl;

                // . set the relative pose
                candidate.relative_pose = candidate_pose;

                // std::cout << "\tEvaluating candidate pose: " << candidate_pose.loc.x() << ", " << candidate_pose.loc.y() << ", " << candidate_pose.angle << std::endl;

                // . use motion model to fill in the p_motion
                candidate.p_motion = motion_model.evaluate(Eigen::Vector3d(candidate_pose.loc.x(), candidate_pose.loc.y(), candidate_pose.angle));

                // TODO now translate the scan to overlay (should already be rotated)
                // transformPoints(cloud, Pose(odom.loc.x() * odom.angle, odom.loc.y() * odom.angle, 0));
                auto cloud {points};
                Eigen::Matrix3f transform {pose2Transform(candidate_pose)};
                // std::cout << "\tProduced cloud transform:\n" << transform << std::endl;
                transformPoints(cloud, transform);

                // . compute scan similarity
                double p_scan {};
                for (const auto &point : cloud) {
                    p_scan += ref->evaluate(point);
                }
                p_scan /= cloud.size();
                candidate.p_scan = p_scan;

                // . compute the combined probability
                candidate.p = PROBABILITY_WEIGHT * candidate.p_scan * candidate.p_motion;
                
                // . add the candidate to the output
                candidates->push_back(candidate);

                // . now add visualization stuff here
                // // draw reference cloud
                // for (const auto &point : ref_points) {
                //     visualization::DrawPoint(point, 0xF633FF, vis_msg_);
                // }

                // // draw projected cloud
                // for (const auto &point : cloud) {
                //     visualization::DrawPoint(point, 0x33FF50, vis_msg_);
                // }

                // // draw pose
                // visualization::DrawParticle(candidate.relative_pose.loc, candidate.relative_pose.angle, vis_msg_);

                // // print info to terminal
                // std::cout << "p_scan: " << candidate.p_scan << ", p_motion: " << candidate.p_motion << ", p: " << candidate.p << std::endl;

                // // publish msg
                // vis_msg_.header.stamp = ros::Time::now();
                // vis_pub_.publish(vis_msg_);

                // // wait for input
                // std::string input;
                // std::getline(std::cin, input);

                // // clear visualization
                // visualization::ClearVisualizationMsg(vis_msg_);
            }
        }
    }
    return candidates;
}

// -------------------------------------------
// * Covariance
// -------------------------------------------

Eigen::Matrix3d SLAM::calculateCovariance(std::shared_ptr<std::vector<Candidate>> &candidates)
{
    Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
    Eigen::Vector3d u = Eigen::Vector3d::Zero();
    double s = 0;
    for (auto &candidate : *candidates){
        Eigen::Vector3d x_i(candidate.relative_pose.loc.x(), candidate.relative_pose.loc.y(), candidate.relative_pose.angle); // I know its ugly let me live steven
        K += x_i*x_i.transpose()*candidate.p_motion*candidate.p_scan; //^
        u += x_i*candidate.p_motion*candidate.p_scan;
        s += candidate.p_motion*candidate.p_scan;
    }

    return 1/s*K-(1/(s*s))*u*u.transpose();
}

// -------------------------------------------
// * Transforms
// -------------------------------------------

Eigen::Matrix3f SLAM::getTransformChain(int ind2, int ind1)
{
    // std::cout << "looking up transform between " << ind2 << " and " << ind1 << std::endl;
    if (ind2 == ind1) {
        return Eigen::Matrix3f::Identity();
    }

    // Defaults to start at the 0th index if only 1 index input. 
    if (static_cast<int>(chain_.size()) <= ind2) {
        return Eigen::Matrix3f::Identity();
    }

    // Accumulate the transforms between the two states
    Eigen::Matrix3f transform;
    transform << cos((chain_[ind1+1]->relative_pose.angle)), -sin(chain_[ind1+1]->relative_pose.angle), chain_[ind1+1]->relative_pose.loc.x(),
                    sin((chain_[ind1+1]->relative_pose.angle)), cos(chain_[ind1+1]->relative_pose.angle), chain_[ind1+1]->relative_pose.loc.y(),
                    0, 0, 1;
    
    if (!(ind2 - ind1 > 1)) {
        return transform;
    }

    for (int i = ind1+2; i <= ind2; i++) {
        Eigen::Matrix3f chain_transform;
        chain_transform << cos((chain_[i]->relative_pose.angle)), -sin(chain_[i]->relative_pose.angle), chain_[i]->relative_pose.loc.x(),
                            sin((chain_[i]->relative_pose.angle)), cos(chain_[i]->relative_pose.angle), chain_[i]->relative_pose.loc.y(),
                            0, 0, 1;
        transform = transform * chain_transform;
    }

    return transform;
}

Eigen::Matrix3f SLAM::pose2Transform(const Pose &pose)
{
    Eigen::Matrix3f transform;
    transform << cos(pose.angle), -sin(pose.angle), pose.loc.x(),
                    sin(pose.angle), cos(pose.angle), pose.loc.y(),
                    0, 0, 1;
    return transform;
}

void SLAM::transformPoses(std::vector<Pose> &poses, const Eigen::Matrix3f &T)
{
    for (auto &pose : poses) {
        transformPose(pose, T);
    }
}

void SLAM::transformPose(Pose &pose, const Eigen::Matrix3f &T)
{
    Eigen::Matrix3f m;
    m << cos(pose.angle), -1 * sin(pose.angle), pose.loc.x(),
        sin(pose.angle), cos(pose.angle), pose.loc.y(),
        0, 0, 1;
    auto result {T * m};
    pose.loc.x() = result(0, 2);
    pose.loc.y() = result(1, 2);
    pose.angle = atan2(result(1, 0), result(0, 0));
}

void SLAM::transformPoses(std::vector<Pose> &poses, const Pose &pose)
{
    // simple case for efficiently doing transforms without rotation
    if (pose.angle == 0) {
        if (pose.loc.x() == 0) {
            for (auto &p : poses) {
                p.loc.y() += pose.loc.y();
            }
        } else if (pose.loc.y() == 0) {
            for (auto &p : poses) {
                p.loc.x() += pose.loc.x();
            }
        } else {
            for (auto &p : poses) {
                p.loc.x() += pose.loc.x();
                p.loc.y() += pose.loc.y();
            }
        }
        return;
    }

    // doing full transformations
    Eigen::Matrix3f transform;
    transform << cos(pose.angle), -sin(pose.angle), pose.loc.x(),
                    sin(pose.angle), cos(pose.angle), pose.loc.y(),
                    0, 0, 1;
    transformPoses(poses, transform);
}

void SLAM::transformPose(Pose &p, const Pose &pose)
{
    // simple case for efficiently doing transforms without rotation
    if (pose.angle == 0) {
        if (pose.loc.x() == 0) {
            p.loc.y() += pose.loc.y();
        } else if (pose.loc.y() == 0) {
            p.loc.x() += pose.loc.x();
        } else {
            p.loc.x() += pose.loc.x();
            p.loc.y() += pose.loc.y();
        }
        return;
    }

    // doing full transformations
    Eigen::Matrix3f transform;
    transform << cos(pose.angle), -sin(pose.angle), pose.loc.x(),
                    sin(pose.angle), cos(pose.angle), pose.loc.y(),
                    0, 0, 1;
    transformPose(p, transform);
}

void SLAM::transformPoints(std::vector<Eigen::Vector2f> &points, const Eigen::Matrix3f &T)
{
    for (auto &point : points) {
        transformPoint(point, T);
    }
}

void SLAM::transformPoint(Eigen::Vector2f &point, const Eigen::Matrix3f &T)
{
    Eigen::Vector3f m(point.x(), point.y(), 1);
    auto result {T * m};
    point.x() = result.x();
    point.y() = result.y();
}

void SLAM::transformPoints(std::vector<Eigen::Vector2f> &points, const Pose &pose)
{
    // simple case for efficiently doing transforms without rotation
    if (pose.angle == 0) {
        if (pose.loc.x() == 0) {
            for (auto &point : points) {
                point.y() += pose.loc.y();
            }
        } else if (pose.loc.y() == 0) {
            for (auto &point : points) {
                point.x() += pose.loc.x();
            }
        } else {
            for (auto &point : points) {
                point.x() += pose.loc.x();
                point.y() += pose.loc.y();
            }
        }
        return;
    }

    // doing full transformations
    Eigen::Matrix3f transform;
    transform << cos(pose.angle), -sin(pose.angle), pose.loc.x(),
                    sin(pose.angle), cos(pose.angle), pose.loc.y(),
                    0, 0, 1;
    transformPoints(points, transform);
}

void SLAM::transformPoint(Eigen::Vector2f &point, const Pose &pose)
{
    // simple case for efficiently doing transforms without rotation
    if (pose.angle == 0) {
        if (pose.loc.x() == 0) {
            point.y() += pose.loc.y();
        } else if (pose.loc.y() == 0) {
            point.x() += pose.loc.x();
        } else {
            point.x() += pose.loc.x();
            point.y() += pose.loc.y();
        }
        return;
    }

    // doing full transformations
    Eigen::Matrix3f transform;
    transform << cos(pose.angle), -sin(pose.angle), pose.loc.x(),
                    sin(pose.angle), cos(pose.angle), pose.loc.y(),
                    0, 0, 1;
    transformPoint(point, transform);
}

}  // namespace slam
