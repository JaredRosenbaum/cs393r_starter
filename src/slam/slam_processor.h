#pragma once

#include <vector>
#include <memory>
#include <eigen3/Eigen/Dense>
#include "motion_model.hpp"
#include "rasterization.hpp"

#include "parameters.h"

namespace slam_processing {

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
    // int child;
    Pose relative_pose;
    Eigen::Matrix3d relative_covariance;
}; // struct NonsequentialNode

struct SequentialNode {
    const int id;
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
        lookup_table = std::make_shared<rasterization::LookupTable>(points, 0.03f, 0.1f);
    }
}; // struct SequentialNode

class Processor
{
public:
    Processor(const int depth);

    void update(
        Pose &odom,
        std::vector<Eigen::Vector2f> &cloud);

private:
    const int depth_;
    std::vector<std::shared_ptr<SequentialNode>> chain_;

    void updatePairwiseSequential(
        std::shared_ptr<SequentialNode> &new_node,
        std::shared_ptr<SequentialNode> &existing_node);
    
    void updatePairwiseNonsequential(
        std::shared_ptr<SequentialNode> &new_node,
        std::shared_ptr<SequentialNode> &existing_node);

    std::shared_ptr<std::vector<Candidate>> generateCandidates(
        Pose odom,
        std::vector<Eigen::Vector2f> points,
        std::shared_ptr<rasterization::LookupTable> &ref);
    
    Eigen::Matrix3d calculateCovariance(std::shared_ptr<std::vector<Candidate>> &candidates);
    // Eigen::Matrix3d calculateCovariance(std::vector<Candidate> candidates);
    
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

}; // class Processor

} // namespace slam_processing

