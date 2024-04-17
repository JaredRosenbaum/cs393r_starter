#include "slam_processor.h"
#include <iostream>

namespace slam_processing {

// -------------------------------------------
// * Constructor
// -------------------------------------------

Processor::Processor(const int depth)
: depth_(depth)
{

}

// -------------------------------------------
// * Primary Update
// -------------------------------------------

void Processor::update(
    Pose &odom,
    std::vector<Eigen::Vector2f> &cloud)
{
    std::cout << "=====================================" << std::endl;

    // . create new sequential state
    // transform cloud into robot frame (if not already done)
    // add its id (just position in the chain)
    // set its raw odometry (update from last cycle)
    // set the cloud (should already be transformed into the robot localization frame)
    // create a lookup table using the cloud
    // and set up the non sequential nodes? or just keep as nullptr?
    auto new_state {std::make_shared<SequentialNode>(chain_.size(), odom, cloud)};
    // new_state->lookup_table->exportAsPPM("/home/dev/cs393r_starter/images/test.ppm"); // ! this may be useful for debugging but it's like 10x slower to do

    // compare with previous state and populate pose and covariance
    if (!chain_.empty()) {
        updatePairwiseSequential(new_state, chain_[chain_.size() - 1]);
    } else {
        new_state->relative_pose = Pose(0, 0, 0);
        new_state->relative_covariance = Eigen::Matrix3d::Zero();
    }

    std::cout << "\nPose:\n" << new_state->relative_pose.loc.x() << ", " << new_state->relative_pose.loc.y() << ", " << new_state->relative_pose.angle << "\nCov:\n" << new_state->relative_covariance << "\n" << std::endl;

    // and add it to the chain
    chain_.push_back(new_state);
    std::cout << "State " << new_state->id << " added to chain!" << std::endl;

    // . now create all non-sequential states
    // for each comparison
    for (int i = 0; i < depth_; i++) {
        // updatePairwiseNonsequential();

        // compute the relative odometry and store it

        // transform the cloud to be in that frame

        // generate perturbations around the new odom location

        // for each perturbation
            // compute scan similarity

            // compute motion model probability
        
        // now compute the covariance and store it 

    }
}

// -------------------------------------------
// * Updating Pairs
// -------------------------------------------

void Processor::updatePairwiseSequential(
    std::shared_ptr<SequentialNode> &new_node,
    std::shared_ptr<SequentialNode> &existing_node)
{
    const auto ref {*existing_node};
    std::cout << "Updating new sequential node!" << std::endl;

    // generate candidates
    // transform point cloud around them
    // compute odometry difference
    // get scan similarity
    // evaluate motion model
    auto candidates {generateCandidates(new_node->raw_odometry, new_node->points, existing_node->lookup_table)};

    // TODO make sure transforms are generated correctly
    for (const auto &candidate : *candidates) {
        std::cout << "\t" << candidate.p_motion << ", " << candidate.p_scan << ", " << candidate.p << std::endl;
    }

    auto covariance {calculateCovariance(candidates)};

    auto best_pose {(*candidates)[0].relative_pose};
    double best_prob {(*candidates)[0].p};
    for (std::size_t i = 1; i < candidates->size(); i++) {
        if ((*candidates)[i].p > best_prob) {
            best_prob = (*candidates)[i].p;
            best_pose = (*candidates)[i].relative_pose;
        }
    }

    // set them in the new node
    new_node->relative_covariance = covariance;
    new_node->relative_pose = best_pose;

    // std::cout << "Pose:\n" << new_node->relative_pose.loc.x() << ", " << new_node->relative_pose.loc.y() << ", " << new_node->relative_pose.angle << "\nCov:\n" << new_node->relative_covariance << std::endl;
}

void Processor::updatePairwiseNonsequential(
    std::shared_ptr<SequentialNode> &new_node,
    std::shared_ptr<SequentialNode> &existing_node)
{

}

// -------------------------------------------
// * Generating Candidates
// -------------------------------------------

std::shared_ptr<std::vector<Candidate>> Processor::generateCandidates(
    Pose odom,
    std::vector<Eigen::Vector2f> points,
    std::shared_ptr<rasterization::LookupTable> &ref)
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

                // . use motion model to fill in the p_motion
                candidate.p_motion = motion_model.evaluate(Eigen::Vector3d(candidate_pose.loc.x(), candidate_pose.loc.y(), candidate_pose.angle));

                // TODO now translate the scan to overlay (should already be rotated)
                // transformPoints(cloud, Pose(odom.loc.x() * odom.angle, odom.loc.y() * odom.angle, 0));
                auto cloud {points};
                Eigen::Matrix3f transform {pose2Transform(candidate_pose)};
                transformPoints(cloud, transform);

                // . compute scan similarity
                double p_scan {};
                for (const auto &point : cloud) {
                    p_scan += ref->evaluate(point);
                }
                p_scan /= cloud.size();
                candidate.p_scan = p_scan;

                // . compute the combined probability
                candidate.p = candidate.p_scan * candidate.p_motion;
                
                // . add the candidate to the output
                candidates->push_back(candidate);

                // TODO how can I visualize this to see if it's right?
            }
        }
    }
    return candidates;
}

// -------------------------------------------
// * Covariance
// -------------------------------------------

Eigen::Matrix3d Processor::calculateCovariance(std::shared_ptr<std::vector<Candidate>> &candidates)
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

// Eigen::Matrix3d Processor::calculateCovariance(std::vector<Candidate> candidates){
//     Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
//     Eigen::Vector3d u = Eigen::Vector3d::Zero();
//     double s = 0;
//     for (auto &candidate : candidates){
//         Eigen::Vector3d x_i(candidate.relative_pose.loc.x(), candidate.relative_pose.loc.y(), candidate.relative_pose.angle); // I know its ugly let me live steven
//         K += x_i*x_i.transpose()*candidate.p_motion*candidate.p_scan; //^
//         u += x_i*candidate.p_motion*candidate.p_scan;
//         s += candidate.p_motion*candidate.p_scan;
//     }

//     return 1/s*K-(1/(s*s))*u*u.transpose();
// }

// -------------------------------------------
// * Transforms
// -------------------------------------------

Eigen::Matrix3f Processor::getTransformChain(int ind2, int ind1) 
{
    // Defaults to start at the 0th index if only 1 index input. 
    if (static_cast<int> (chain_.size()) <= ind2) {
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

Eigen::Matrix3f Processor::pose2Transform(const Pose &pose)
{
    Eigen::Matrix3f transform;
    transform << cos(pose.angle), -sin(pose.angle), pose.loc.x(),
                    sin(pose.angle), cos(pose.angle), pose.loc.y(),
                    0, 0, 1;
    return transform;
}

void Processor::transformPoses(std::vector<Pose> &poses, const Eigen::Matrix3f &T)
{
    for (auto &pose : poses) {
        transformPose(pose, T);
    }
}

void Processor::transformPose(Pose &pose, const Eigen::Matrix3f &T)
{
    Eigen::Matrix3f m;
    m << cos(pose.angle), -1 * sin(pose.angle), pose.loc.x(),
        sin(pose.angle), cos(pose.angle), pose.loc.y(),
        0, 0, 1;
    auto result {T * m};
    pose.loc.x() = result(0, 2);
    pose.loc.y() = result(1, 2);
    pose.angle = atan2(result(1, 0), result(0, 0)); // TODO check this, make sure radians too
}

void Processor::transformPoses(std::vector<Pose> &poses, const Pose &pose)
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

void Processor::transformPose(Pose &p, const Pose &pose)
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

void Processor::transformPoints(std::vector<Eigen::Vector2f> &points, const Eigen::Matrix3f &T)
{
    for (auto &point : points) {
        transformPoint(point, T);
    }
}

void Processor::transformPoint(Eigen::Vector2f &point, const Eigen::Matrix3f &T)
{
    Eigen::Vector3f m(point.x(), point.y(), 0);
    auto result {T * m};
    point.x() = result.x();
    point.y() = result.y();
}

void Processor::transformPoints(std::vector<Eigen::Vector2f> &points, const Pose &pose)
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

void Processor::transformPoint(Eigen::Vector2f &point, const Pose &pose)
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

} // namespace slam_processing
