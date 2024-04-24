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
    depth_(POSE_GRAPH_CONNECTION_DEPTH),
    gtsam_timer_(0),
    iteration_counter_(0) {}

void SLAM::CreateVisPublisher(ros::NodeHandle* n) {
    vis_pub_ = n->advertise<amrl_msgs::VisualizationMsg>("visualization", 1);
    vis_msg_ = visualization::NewVisualizationMessage("map", "slam");
}

void SLAM::InitializePose(const Eigen::Vector2f& loc, const float angle) {
  current_pose_.loc = loc;
  current_pose_.angle = angle;
  starting_pose_ = std::make_shared<gtsam::Pose2>(loc.x(), loc.y(), angle);
  visualization::ClearVisualizationMsg(vis_msg_);
  odom_initialized_ = false;
}

void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) {
  // Return the latest pose estimate of the robot.
  if (!(chain_.size() > 0)) {
    const auto pose {transformPoseCopy(odom_change_, current_pose_)};
    *loc = pose.loc;
    *angle = pose.angle;
    return;
  }

  const auto last_chain_pose {Pose(chain_[chain_.size() - 1]->est_pose.x(), chain_[chain_.size() - 1]->est_pose.y(), chain_[chain_.size() - 1]->est_pose.theta())};
  Pose current {transformPoseCopy(odom_change_, last_chain_pose)};
  *loc = current.loc;
  *angle = current.angle;
}

void SLAM::ObserveLaser(const vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max) {
    // A new laser scan has been observed. Decide whether to add it as a pose
    // for SLAM. If decided to add, align it to the scan from the last saved pose,
    // and save both the scan and the optimized pose.

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
    }

    // Publish point cloud visualization
    vis_msg_.header.stamp = ros::Time::now();
    vis_pub_.publish(vis_msg_);

    // update SLAM
    iterateSLAM(odom_change_, point_cloud);

    // Clear motion model flag
    ready_for_slam_update_ = false;
}

void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
    static float odom_translation_change;
    static float odom_rotation_change;

    if (starting_pose_ == nullptr) {
        return;
    }

    // Disregard first odometry reading, set pose variables
    if (!odom_initialized_) {
        odom_translation_change = 0;
        odom_rotation_change = 0;

        reference_odom_pose_.loc = odom_loc;
        reference_odom_pose_.angle = odom_angle;

        odom_initialized_ = true;
        return;
    }
    else {
        // Calculate pose change from odometry reading
        float odom_translation = (odom_loc - prev_odom_pose_.loc).norm();
        float odom_rotation = std::abs(AngleDiff(odom_angle, prev_odom_pose_.angle));

        // Ignore unrealistic jumps in odometry
        if (odom_translation < 1.0 && odom_rotation < M_PI_4) {
            // Keep track of movement from odometry readings to estimate how far the robot has moved
            odom_translation_change += odom_translation;
            odom_rotation_change += odom_rotation;

            // updating odometry change in odom frame for use in later algorithm
            Eigen::Matrix3f transform;
            transform << cos(reference_odom_pose_.angle), -sin(reference_odom_pose_.angle), reference_odom_pose_.loc.x(),
                            sin(reference_odom_pose_.angle), cos(reference_odom_pose_.angle), reference_odom_pose_.loc.y(),
                            0, 0, 1;
            odom_change_ = transformPoseCopy(Pose(odom_loc.x(), odom_loc.y(), odom_angle), transform.inverse());

            // Proceed with preparing a motion model when meeting threshold
            if ((odom_translation_change > ODOM_TRANSLATION_THRESHOLD || odom_rotation_change > ODOM_ROTATION_THRESHOLD) && !ready_for_slam_update_) {
                ready_for_slam_update_ = true;

                // Update reference pose
                reference_odom_pose_.loc = odom_loc;
                reference_odom_pose_.angle = odom_angle;

                // Reset odom change variables
                odom_translation_change = 0;
                odom_rotation_change = 0;
            }
        }
    }

    // Update previous odometry reading
    prev_odom_pose_.loc = odom_loc;
    prev_odom_pose_.angle = odom_angle;
}

std::vector<Eigen::Vector2f> SLAM::GetMap() {
    std::vector<Eigen::Vector2f> map;
    // Reconstruct the map as a single aligned point cloud from all saved poses
    // and their respective scans.
    for (int i = 0; i < static_cast<int>(chain_.size()); i++) {
        // transform cloud
        auto viz_points {chain_[i]->points};
        transformPoints(viz_points, Pose(chain_[i]->est_pose.x(), chain_[i]->est_pose.y(), chain_[i]->est_pose.theta()));

        // add to map
        map.insert(map.end(), viz_points.begin(), viz_points.end());
    }
    return map;
}

// -------------------------------------------
// * Primary Update
// -------------------------------------------

void SLAM::iterateSLAM(
    Pose &odom,
    std::vector<Eigen::Vector2f> &cloud)
{
    auto start_time {std::chrono::high_resolution_clock::now()};
    std::cout << "\n\n=====================================" << std::endl;
    std::cout << "UPDATE " << iteration_counter_ << std::endl;
    std::cout << "=====================================" << std::endl;

    // . add new sequential state
    auto new_state {std::make_shared<SequentialNode>(chain_.size(), odom, cloud)};
    std::cout << "Creating new state with ID " << new_state->id << " and " << cloud.size() << " points." << std::endl;

    // compare with previous state and populate pose and covariance
    if (!chain_.empty()) { // if data immediately before this exists
        auto start_time {std::chrono::high_resolution_clock::now()};
        int index {static_cast<int>(chain_.size()) - 1};

        // perform the update (results same for sequenial and non-sequential scans, but the way they are stored is different)
        auto csm_results {pairwiseComparison(new_state, chain_[index])};

        // TODO how to handle bad comparison here?
        // if (std::holds_alternative<bool>(csm_results)) {
        //     std::cerr << "Invalid CSM results for " << new_state->id << ". Unable to add to chain." << std::endl;

        // } else {
            // convert relative pose to absolute pose using previous absolute pose
            auto valid_results {std::get<std::pair<gtsam::Pose2, gtsam::Matrix>>(csm_results)}; //. shouldnt be necessary

            new_state->rel_pose = valid_results.first;
            new_state->rel_cov = valid_results.second;
            new_state->est_pose = transformPoseCopy(gtsam::Pose2(odom.loc.x(), odom.loc.y(), odom.angle), chain_[static_cast<int>(chain_.size()) - 1]->est_pose);

            auto end_time {std::chrono::high_resolution_clock::now()};
            auto time_duration {std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time)};
            std::cout << ">> Finished sequential pose update in " << time_duration.count() << "us" << std::endl;
            
            // add to the chain
            chain_.push_back(new_state);
            std::cout << "State " << new_state->id << " added to chain!" << std::endl;
        // }
    } else { // if no previous data in the chain to compare to
        new_state->rel_pose = gtsam::Pose2(0, 0, 0);
        new_state->rel_cov = gtsam::Matrix(Eigen::Matrix3d::Zero());
        new_state->est_pose = transformPoseCopy(gtsam::Pose2(odom.loc.x(), odom.loc.y(), odom.angle), *starting_pose_);

        // add to the chain
        chain_.push_back(new_state);
        std::cout << "State " << new_state->id << " added to chain!" << std::endl;
    }

    std::cout << "Abs:\n" << new_state->est_pose << "\nRel:\n" << new_state->rel_pose << "\nCov:\n" << new_state->rel_cov << std::endl;

    // . now create all non-sequential states
    // for each comparison
    for (int i = 1; i <= depth_; i++) {
        int index {(static_cast<int>(chain_.size()) - 1) - i - 1};
        if (index < 0) {continue;}
        // std::cout << "\nAdding nonsequential relation with " << index << std::endl;

        // get pose and covariance
        auto csm_results {pairwiseComparison(new_state, chain_[index])};

        // if (std::holds_alternative<bool>(csm_results)) {
        //     std::cerr << "Invalid CSM results for " << new_state->id << ". Unable to add to chain." << std::endl;
        // } else {
            auto valid_results {std::get<std::pair<gtsam::Pose2, gtsam::Matrix>>(csm_results)}; //.shouldnt be necessary

            // store them appropriately
            NonsequentialNode node;
            node.parent = chain_[index]->id;
            node.rel_odom = transformPoseCopy(odom, chain_[index]->rel_odom);
            node.rel_pose = valid_results.first;
            node.rel_cov = valid_results.second;
            node.est_pose = transformPoseCopy(valid_results.first, chain_[index]->est_pose);
            new_state->nodes.push_back(node);

            // std::cout << "Abs:\n" << node.est_pose << "\nPose:\n" << node.rel_pose << "\nCov:\n" << node.rel_cov << std::endl;
        // }
    }

    // . perform pose graph optimization using GTSAM
    gtsam_timer_++;
    if (gtsam_timer_ == GTSAM_FREQUENCY) {
        std::cout << "Running through GTSAM!" << std::endl;
        optimizeChain();
        gtsam_timer_ = 0;
    }

    // . now add visualization stuff here for debugging
    visualization::ClearVisualizationMsg(vis_msg_);

    for (int i = 0; i < static_cast<int>(chain_.size()); i++) {
        // transform cloud
        auto viz_points {chain_[i]->points};
        transformPoints(viz_points, Pose(chain_[i]->est_pose.x(), chain_[i]->est_pose.y(), chain_[i]->est_pose.theta()));

        // visualize cloud
        for (const auto &point : viz_points) {
            visualization::DrawPoint(point, 0xC0C0C0, vis_msg_);
        }
        // F633FF
        
        // visualize pose
        visualization::DrawParticle(Eigen::Vector2f(chain_[i]->est_pose.x(), chain_[i]->est_pose.y()), chain_[i]->est_pose.theta(), vis_msg_);

        // and visualize all non-sequential ones to make sure they're in the right spot
        for (const auto &node : chain_[i]->nodes) {
            visualization::DrawParticle(Eigen::Vector2f(node.est_pose.x(), node.est_pose.y()), node.est_pose.theta(), vis_msg_);
        }
    }

    // visualize the selected cloud from CSM
    auto viz_points {new_state->points};
    transformPoints(viz_points, Pose(new_state->est_pose.x(), new_state->est_pose.y(), new_state->est_pose.theta()));
    for (const auto &point : viz_points) {
        visualization::DrawPoint(point, 0x33FF4A, vis_msg_);
    }

    vis_msg_.header.stamp = ros::Time::now();
    vis_pub_.publish(vis_msg_);

    auto end_time {std::chrono::high_resolution_clock::now()};
    auto time_duration {std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time)};
    std::cout << ">> Finished SLAM update in " << time_duration.count() << "us" << std::endl;
    iteration_counter_++;
}

// -------------------------------------------
// * Pose Graph Optimization
// -------------------------------------------

void SLAM::optimizeChain()
{
    // create GTSAM graph and initial estimate
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial_estimate;

    // iterate over chain and add all points and initial estimates
    // need to associate poses with unique indexes for recovery
    int graph_id {0};

    auto priorNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.1, 0.1, 0.03));
    graph.addPrior(0, gtsam::Pose2(0, 0, 0), priorNoise);
    initial_estimate.insert(graph_id, chain_[0]->est_pose);

    // start by adding sequential poses to graph
    for (std::size_t i = 1; i < chain_.size(); i++) {
        graph_id++;
        graph.add(gtsam::BetweenFactor<gtsam::Pose2>(
            graph_id - 1,
            graph_id,
            chain_[i]->rel_pose,
            gtsam::noiseModel::Gaussian::Covariance(chain_[i]->rel_cov)
        ));
        initial_estimate.insert(graph_id, chain_[i]->est_pose);
    }

    // now add in the non-sequential poses
    for (const auto &node : chain_) {
        for (const auto &n : node->nodes) {
            graph_id++;
            graph.add(gtsam::BetweenFactor<gtsam::Pose2>(
                n.parent,
                graph_id,
                n.rel_pose,
                gtsam::noiseModel::Gaussian::Covariance(n.rel_cov)
            ));
            initial_estimate.insert(graph_id, n.est_pose);
        }
    }

    // print the graph
    // graph.print();
    graph.saveGraph("/home/dev/cs393r_starter/graphs/gtsam.dot");

    // optimize the graph
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_estimate);
    gtsam::Values optimized_result = optimizer.optimize();

    // iterate over the data and update it
    int new_graph_id {0};
    for (std::size_t i = 1; i < chain_.size(); i++) {
        new_graph_id++;
        chain_[i]->est_pose = transformPoseCopy(optimized_result.at<gtsam::Pose2>(new_graph_id), chain_[0]->est_pose);
    }

    for (auto &node : chain_) {
        for (auto &n : node->nodes) {
            new_graph_id++;
            n.est_pose = transformPoseCopy(optimized_result.at<gtsam::Pose2>(new_graph_id), chain_[0]->est_pose);
        }
    }
}

// -------------------------------------------
// * Updating Pairs
// -------------------------------------------

std::variant<bool, std::pair<gtsam::Pose2, gtsam::Matrix>> SLAM::pairwiseComparison(
    std::shared_ptr<SequentialNode> &new_node,
    std::shared_ptr<SequentialNode> &existing_node)
{
    // check if nodes are not sequential; if not, need to compute the relative odometry
    auto rel_odom {new_node->rel_odom};
    if (new_node->id != existing_node->id + 1) {
        Pose rel_pose(0, 0, 0);
        for (int i = 0; i < new_node->id - existing_node->id - 1; i++) {
            transformPose(rel_pose, chain_[existing_node->id + i]->rel_odom);
        }
        transformPose(rel_pose, new_node->rel_odom);
        rel_odom = rel_pose;
    }
    
    // generate candidates
    auto candidates {generateCandidates(rel_odom, new_node->points, existing_node->lookup_table)};

    // calculate covariance
    Eigen::Matrix3d cov_matrix;
    Pose best_pose;
    auto covariance {calculateCovariance(candidates)};
    if (std::holds_alternative<bool>(covariance)) {
        auto motion_model {motion_model::MultivariateMotionModel(Eigen::Vector3d(rel_odom.loc.x(), rel_odom.loc.y(), rel_odom.angle))};
        cov_matrix = motion_model.get_covariance();
        std::cout << "Bad covariance detected for " << new_node->id << " and " << existing_node->id << std::endl;
        std::cout << "Overriding with pure odometry covariance." << std::endl;

        best_pose = rel_odom;
    }
    else{
        cov_matrix = {std::get<Eigen::Matrix3d>(covariance)};
        // and get the best pose
        best_pose = (*candidates)[0].relative_pose;
        double best_prob {(*candidates)[0].p};
        for (std::size_t i = 1; i < candidates->size(); i++) {
            if ((*candidates)[i].p > best_prob) {
                best_prob = (*candidates)[i].p;
                best_pose = (*candidates)[i].relative_pose;
            }
        }
    }

    return std::make_pair<gtsam::Pose2, gtsam::Matrix>(gtsam::Pose2(best_pose.loc.x(), best_pose.loc.y(), best_pose.angle), gtsam::Matrix(cov_matrix));
}

// -------------------------------------------
// * Generating Candidates
// -------------------------------------------

std::shared_ptr<std::vector<Candidate>> SLAM::generateCandidates(
    Pose odom,
    std::vector<Eigen::Vector2f> points,
    std::shared_ptr<rasterization::LookupTable> &ref)
{
    auto candidates {std::make_shared<std::vector<Candidate>>()};

    // create motion model
    auto motion_model {motion_model::MultivariateMotionModel(Eigen::Vector3d(odom.loc.x(), odom.loc.y(), odom.angle))};

    // calculate periods of thresholds based on given odom pose.
    // used for dynamically allocation larger range of candidates for relative poses (which are further)
    const int translation_periods = std::round(odom.loc.norm() / ODOM_TRANSLATION_THRESHOLD);
    const int rotation_periods = std::round(std::abs(odom.angle) / ODOM_ROTATION_THRESHOLD);
    const int periods = std::max(translation_periods, rotation_periods);

    // iterate over candidates
    const int x_iterations = std::ceil(MOTION_MODEL_X_LIMIT / MOTION_MODEL_X_RESOLUTION);
    const int y_iterations = std::ceil(MOTION_MODEL_Y_LIMIT / MOTION_MODEL_Y_RESOLUTION);
    const int theta_iterations = std::ceil(MOTION_MODEL_THETA_LIMIT / MOTION_MODEL_THETA_RESOLUTION);
    // const int x_iterations = std::round(std::ceil(MOTION_MODEL_X_LIMIT / MOTION_MODEL_X_RESOLUTION) * (1 + (float)periods / DYNAMIC_ADJUSTMENT_DIVIDER));
    // const int y_iterations = std::round(std::ceil(MOTION_MODEL_Y_LIMIT / MOTION_MODEL_Y_RESOLUTION) * (1 + (float)periods / DYNAMIC_ADJUSTMENT_DIVIDER));
    // const int theta_iterations = std::round(std::ceil(MOTION_MODEL_THETA_LIMIT / MOTION_MODEL_THETA_RESOLUTION) * (1 + (float)periods / DYNAMIC_ADJUSTMENT_DIVIDER));

    // reserving space in candidates
    candidates->reserve(x_iterations * y_iterations * theta_iterations);

    // Triple loop to populate motion model poses with variance in each dimension. This creates a cube of next state possibilities
    for (int theta_i = -theta_iterations; theta_i <= theta_iterations; theta_i++) {
        Candidate candidate;
        for (int  y_i = -y_iterations; y_i <= y_iterations; y_i++) {
            for (int x_i = -x_iterations; x_i <= x_iterations; x_i++) {
                
                // . generate a candidate pose WRT to the body
                auto candidate_pose {Pose(
                        static_cast<float>(x_i * MOTION_MODEL_X_RESOLUTION * periods), 
                        static_cast<float>(y_i * MOTION_MODEL_Y_RESOLUTION * periods), 
                        static_cast<float>(theta_i * MOTION_MODEL_THETA_RESOLUTION * periods))
                };

                // . transform it back to the last frame (from odom)
                transformPose(candidate_pose, odom);

                // . set the relative pose
                candidate.relative_pose = candidate_pose;

                // . use motion model to fill in the p_motion
                candidate.p_motion = motion_model.evaluate(Eigen::Vector3d(candidate_pose.loc.x(), candidate_pose.loc.y(), candidate_pose.angle));

                // . transform the points to the candidate frame
                auto cloud {points};
                Eigen::Matrix3f transform {pose2Transform(candidate_pose)};
                transformPoints(cloud, transform);

                // . compute scan similarity
                double p_scan {};
                if (cloud.size() > 0) {
                    for (const auto &point : cloud) {
                        p_scan += ref->evaluate(point);
                    }
                    p_scan /= cloud.size();
                }
                candidate.p_scan = p_scan;

                // . compute the combined probability
                candidate.p = PROBABILITY_WEIGHT * candidate.p_scan * candidate.p_motion;
                
                // . add the candidate to the output
                candidates->push_back(candidate);
            }
        }
    }
    return candidates;
}

// -------------------------------------------
// * Covariance
// -------------------------------------------

std::variant<bool, Eigen::Matrix3d> SLAM::calculateCovariance(std::shared_ptr<std::vector<Candidate>> &candidates)
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

    if (s < 1e-9) {
        return false;
    }

    return 1/s*K-(1/(s*s))*u*u.transpose();
}

// -------------------------------------------
// * Transforms
// -------------------------------------------

// & convert pose to transform

Eigen::Matrix3f SLAM::pose2Transform(const Pose &pose)
{
    Eigen::Matrix3f transform;
    transform << cos(pose.angle), -sin(pose.angle), pose.loc.x(),
                    sin(pose.angle), cos(pose.angle), pose.loc.y(),
                    0, 0, 1;
    return transform;
}

// & transform pose

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

void SLAM::transformPose(Pose &pose, const Pose &P)
{
    // simple case for efficiently doing transforms without rotation
    if (P.angle == 0) {
        if (P.loc.x() == 0) {
            pose.loc.y() += P.loc.y();
        } else if (P.loc.y() == 0) {
            pose.loc.x() += P.loc.x();
        } else {
            pose.loc.x() += P.loc.x();
            pose.loc.y() += P.loc.y();
        }
        return;
    }

    // doing full transformations
    Eigen::Matrix3f transform;
    transform << cos(P.angle), -sin(P.angle), P.loc.x(),
                    sin(P.angle), cos(P.angle), P.loc.y(),
                    0, 0, 1;
    transformPose(pose, transform);
}

Pose SLAM::transformPoseCopy(const Pose &pose, const Eigen::Matrix3f &T)
{
    auto p_copy {pose};
    Eigen::Matrix3f m;
    m << cos(p_copy.angle), -1 * sin(p_copy.angle), p_copy.loc.x(),
        sin(p_copy.angle), cos(p_copy.angle), p_copy.loc.y(),
        0, 0, 1;
    auto result {T * m};
    p_copy.loc.x() = result(0, 2);
    p_copy.loc.y() = result(1, 2);
    p_copy.angle = atan2(result(1, 0), result(0, 0));
    return p_copy;
}

Pose SLAM::transformPoseCopy(const Pose &pose, const Pose &P)
{
    // simple case for efficiently doing transforms without rotation
    if (P.angle == 0) {
        auto pose_copy{pose};
        if (P.loc.x() == 0) {
            pose_copy.loc.y() += P.loc.y();
        } else if (P.loc.y() == 0) {
            pose_copy.loc.x() += P.loc.x();
        } else {
            pose_copy.loc.x() += P.loc.x();
            pose_copy.loc.y() += P.loc.y();
        }
        return pose_copy;
    }

    // doing full transformations
    Eigen::Matrix3f transform;
    transform << cos(P.angle), -sin(P.angle), P.loc.x(),
                    sin(P.angle), cos(P.angle), P.loc.y(),
                    0, 0, 1;
    return transformPoseCopy(pose, transform);
}

gtsam::Pose2 SLAM::transformPoseCopy(const gtsam::Pose2 &pose, const Eigen::Matrix3f &T)
{
    Eigen::Matrix3f m;
    m << cos(pose.theta()), -1 * sin(pose.theta()), pose.x(),
        sin(pose.theta()), cos(pose.theta()), pose.y(),
        0, 0, 1;
    auto result {T * m};
    return gtsam::Pose2(result(0, 2), result(1, 2), atan2(result(1, 0), result(0, 0)));
}

gtsam::Pose2 SLAM::transformPoseCopy(const gtsam::Pose2 &pose, const gtsam::Pose2 &P)
{
    // simple case for efficiently doing transforms without rotation
    if (P.theta() == 0) {
        if (P.x() == 0) {
            return gtsam::Pose2(pose.x(), pose.y() + P.y(), pose.theta());
        } else if (P.y() == 0) {
            return gtsam::Pose2(pose.x() + P.x(), pose.y(), pose.theta());
        } else {
            return gtsam::Pose2(pose.x() + P.x(), pose.y() + P.y(), pose.theta());
        }
    }

    // doing full transformations
    Eigen::Matrix3f transform;
    transform << cos(P.theta()), -sin(P.theta()), P.x(),
                    sin(P.theta()), cos(P.theta()), P.y(),
                    0, 0, 1;
    return transformPoseCopy(pose, transform);
}

// & transform poses

void SLAM::transformPoses(std::vector<Pose> &poses, const Eigen::Matrix3f &T)
{
    for (auto &pose : poses) {
        transformPose(pose, T);
    }
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

// & transform point

void SLAM::transformPoint(Eigen::Vector2f &point, const Eigen::Matrix3f &T)
{
    Eigen::Vector3f m(point.x(), point.y(), 1);
    auto result {T * m};
    point.x() = result.x();
    point.y() = result.y();
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

// & transform points 

void SLAM::transformPoints(std::vector<Eigen::Vector2f> &points, const Eigen::Matrix3f &T)
{
    for (auto &point : points) {
        transformPoint(point, T);
    }
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

}  // namespace slam
