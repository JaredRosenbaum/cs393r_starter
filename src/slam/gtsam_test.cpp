#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <boost/optional.hpp>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

using namespace std;
using namespace gtsam;

// class CustomFactor : public NoiseModelFactor2<Pose2, Matrix> {

// private:
//     typedef NoiseModelFactor2<Pose2, Matrix> Base;

// public:
//     CustomFactor(Key poseKey1, Key poseKey2, const Pose2 &measured_relative_pose, const Matrix &covariance)
//     : Base(gtsam::noiseModel::Gaussian::Covariance(covariance), poseKey1, poseKey2), measured_relative_pose_(measured_relative_pose) {}

//     Vector evaluateError(const Pose2 &pose1, const Pose2 &pose2, boost::optional<Matrix&> H1=boost::none, boost::optional<Matrix&> H2=boost::none) const override {
//         Pose2 predicted_relative_pose = pose1.between(pose2, H1, H2);
//         Vector error = predicted_relative_pose.localCoordinates(measured_relative_pose_);
//         return error;
//     }

// private:
//     Pose2 measured_relative_pose_;

// }; // class CustomFactor

// class PoseCovarianceFactor : public NoiseModelFactor2<Pose3, Matrix> {
// private:
//     typedef NoiseModelFactor2<Pose3, Matrix> Base;

// public:
//     PoseCovarianceFactor(Key poseKey1, Key poseKey2, const Pose3& measuredRelativePose, const Matrix& covariance)
//         : Base(gtsam::noiseModel::Gaussian::Covariance(covariance), poseKey1, poseKey2), measuredRelativePose_(measuredRelativePose) {}

//     Vector evaluateError(const Pose3& pose1, const Pose3& pose2, boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) const override {
//         // Calculate error as the difference between the predicted relative pose and the measured relative pose
//         Pose3 predictedRelativePose = pose1.between(pose2, H1, H2);
//         Vector error = predictedRelativePose.localCoordinates(measuredRelativePose_);
//         return error;
//     }

// private:
//     Pose3 measuredRelativePose_;
// };


// int main(int argc, char** argv) {

//     // 1. Create the graph
//     NonlinearFactorGraph graph;
//     noiseModel::Diagonal::shared_ptr priorNoise =
//     noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
//     graph.add(PriorFactor<Pose2>(1, Pose2(0, 0, 0), priorNoise));

//     // 2. Add odometry factors
//     noiseModel::Diagonal::shared_ptr model =
//     noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
//     graph.add(BetweenFactor<Pose2>(1, 2, Pose2(2, 0, 0     ), model));
//     graph.add(BetweenFactor<Pose2>(2, 3, Pose2(2, 0, M_PI_2), model));
//     graph.add(BetweenFactor<Pose2>(3, 4, Pose2(2, 0, M_PI_2), model));
//     graph.add(BetweenFactor<Pose2>(4, 5, Pose2(2, 0, M_PI_2), model));

//     // 3. Add the loop closure constraint
//     graph.add(BetweenFactor<Pose2>(5, 2, Pose2(2, 0, M_PI_2), model));

//     graph.print();

//     Values initialEstimate;
//     initialEstimate.insert(1, Pose2(0.5, 0.0,  0.2   ));
//     initialEstimate.insert(2, Pose2(2.3, 0.1, -0.2   ));
//     initialEstimate.insert(3, Pose2(4.1, 0.1,  M_PI_2));
//     initialEstimate.insert(4, Pose2(4.0, 2.0,  M_PI  ));
//     initialEstimate.insert(5, Pose2(2.1, 2.1, -M_PI_2));
//     initialEstimate.print("\nInitial Estimate:\n"); // print

//     // 4. Optimize the initial values using a Gauss-Newton nonlinear optimizer
//     // The optimizer accepts an optional set of configuration parameters,
//     // controlling things like convergence criteria, the type of linear
//     // system solver to use, and the amount of information displayed during
//     // optimization. We will set a few parameters as a demonstration.
//     GaussNewtonParams parameters;
//     // Stop iterating once the change in error between steps is less than this value
//     parameters.relativeErrorTol = 1e-5;
//     // Do not perform more than N iteration steps
//     parameters.maxIterations = 100;
//     // Create the optimizer ...
//     GaussNewtonOptimizer optimizer(graph, initialEstimate, parameters);
//     // ... and optimize
//     Values result = optimizer.optimize();
//     result.print("Final Result:\n");

//     // 5. Calculate and print marginal covariances for all variables
//     cout.precision(3);
//     Marginals marginals(graph, result);
//     cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl;
//     cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
//     cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;
//     cout << "x4 covariance:\n" << marginals.marginalCovariance(4) << endl;
//     cout << "x5 covariance:\n" << marginals.marginalCovariance(5) << endl;

//     return 0;
// }

int main(int argc, char **argv)
{
    // create inputs
    std::vector<Pose2> robot_poses;
    std::vector<Pose2> landmark_positions;

    int n_poses {10};
    int n_landmarks {3};

    robot_poses.reserve(n_poses);
    landmark_positions.reserve(n_landmarks);

    for (int i = 0; i < n_poses; ++i) {
        double x {i * 2.0};
        double y {i * 1.5};
        double theta {i * 0.1};
        robot_poses.push_back(Pose2(x, y, theta));
    }

    for (int i = 0; i < n_landmarks; ++i) {
        double lx {i * 2.0};
        double ly {i * 1.5};
        landmark_positions.push_back(Pose2(lx, ly, 0.0));
    }

    // create graph
    NonlinearFactorGraph graph;

    // add odometry factors
    // for (int t = 1; t < n_poses; ++t) {
    //     Pose2 odometry_measurement = Pose2(robot_poses[t-1].between(robot_poses[t]));
    //     Matrix odom_covariance = I_3x3;
    //     graph.add(BetweenFactor<Pose2>(t-1, t, odometry_measurement, noiseModel::Gaussian::Covariance(odom_covariance)));
    // }

    // // add scan matching factors
    // for (int t = 0; t < n_poses; ++t) {
    //     for (int i = 0; i < n_landmarks; ++i) {
    //         Pose2 delta = landmark_positions[i].between(robot_poses[t]);
    //         Pose2 scan_match_measurement = Pose2(delta.x(), delta.y(), delta.theta());
    //         Matrix scan_match_covariance = I_3x3;
    //         graph.add(BetweenFactor<Pose2>(t, n_poses + i, scan_match_measurement, noiseModel::Gaussian::Covariance(scan_match_covariance)));
    //     }
    // }

    // now, try adding factors between non-adjacent poses
    const int n_previous_poses {3};
    for (int t = 0; t < n_poses; ++t) {
        // for (int i = t - 1; i >= 0; i--) {
        for (int j = 1; j <= n_previous_poses; ++j) {
            if (t + j >= n_poses) {continue;}
            Pose2 rel_pose {Pose2(robot_poses[t].between(robot_poses[t + j]))};
            Matrix rel_covariance = I_3x3;
            graph.add(BetweenFactor<Pose2>(t, t + j, rel_pose, noiseModel::Gaussian::Covariance(rel_covariance)));
        }
    }

    graph.print();

    std::cout << "Iterating over factors in the graph:" << std::endl;
    for (size_t i = 0; i < graph.size(); ++i) {
        const NonlinearFactor::shared_ptr factor = graph.at(i);
        
        // Check factor type using dynamic_pointer_cast
        if (const BetweenFactor<Pose2>* betweenFactor = dynamic_cast<const BetweenFactor<Pose2>*>(factor.get())) {
            Key key1 = betweenFactor->key1();
            Key key2 = betweenFactor->key2();
            Pose2 measurement = betweenFactor->measured();
            SharedNoiseModel noiseModel = betweenFactor->noiseModel();

            std::cout << "BetweenFactor: Keys(" << key1 << ", " << key2 << "), Measurement(" << measurement << "), Noise(" << noiseModel << ")" << std::endl;
        } else {
            std::cout << "Unknown factor type!" << std::endl;
        }
    }

    // set initial values
    Values initial_estimate;
    
    for (int t = 0; t < n_poses; ++t) {
        initial_estimate.insert(t, robot_poses[t]);
    }

    // for (int i = 0; i < n_landmarks; ++i) {
    //     initial_estimate.insert(n_poses + i, landmark_positions[i]);
    // }

    // perform optimization
    LevenbergMarquardtOptimizer optimizer(graph, initial_estimate);
    Values optimized_result = optimizer.optimize();

    // retrieve output
    cout << "Optimized Poses:" << endl;
    for (int t = 0; t < n_poses; ++t) {
        Pose2 optimized_pose = optimized_result.at<Pose2>(t);
        cout << "Pose " << t << ": x=" << optimized_pose.x() << ", y=" << optimized_pose.y() << ", theta=" << optimized_pose.theta() << endl;
    }

    // cout << "Optimized Landmark Positions:" << endl;
    // for (int i = 0; i < n_landmarks; ++i) {
    //     Pose2 optimized_landmark = optimized_result.at<Pose2>(n_poses + i);
    //     cout << "Landmark " << i << ": x=" << optimized_landmark.x() << ", y=" << optimized_landmark.y() << ", theta=" << optimized_landmark.theta() << endl;
    // }

    return 0;
}
