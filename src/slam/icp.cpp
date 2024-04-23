#include "icp.h"

namespace icp {
using namespace Eigen;

bool performICP(const std::vector<Eigen::Vector2f> &source, const std::vector<Eigen::Vector2f> &target, const int max_iterations, const double tolerance)
{
    Eigen::Matrix3f T {Eigen::Matrix3f::Identity(3, 3)};
    // auto p {Eigen::Vector2f(0, 0)};

    bool converged {false};
    for (int i = 0; i < max_iterations && !converged; ++i) {
        std::cout << "=============" << std::endl;
        std::cout << "i: " << i << std::endl;
        std::cout << T << std::endl;

        // find closest points
        auto source_transformed {transformPoints(source, T)};

        std::vector<int> correspondences;
        findCorrespondences(source_transformed, target, correspondences);

        Eigen::Matrix3f update {computeTransform(source_transformed, target, correspondences)};
        T = update * T;

        std::cout << update << std::endl;

        std::cout << "\t" << update.norm() << ", " << tolerance << std::endl;
        if (update.norm() < tolerance) {
            converged = true;
        }
    }

    if (converged) {
        return true;
    }
    return false;
}

Eigen::Vector2f transformPoint(const Eigen::Vector2f &point, const Eigen::Matrix3f &T)
{
    Eigen::Vector3f m(point.x(), point.y(), 1);
    auto result {T * m};
    return Eigen::Vector2f(result.x(), result.y());
}

std::vector<Eigen::Vector2f> transformPoints(const std::vector<Eigen::Vector2f> &points, const Eigen::Matrix3f &T)
{
    std::vector<Eigen::Vector2f> output;
    output.reserve(points.size());
    for (const auto &point : points) {
        output.push_back(transformPoint(point, T));
    }
    return output;
}

void findCorrespondences(const std::vector<Eigen::Vector2f> &source, const std::vector<Eigen::Vector2f> &target, std::vector<int> &correspondences)
{
    correspondences.clear();
    correspondences.reserve(source.size());
    for (const auto &point : source) {
        double d_min_sqrd {std::numeric_limits<double>::max()};
        int closest_index {-1};

        for (std::size_t i = 0; i < target.size(); ++i) {
            double d_sqrd {(point - target[i]).squaredNorm()};
            if (d_sqrd < d_min_sqrd) {
                d_min_sqrd = d_sqrd;
                closest_index = i;
            }
        }
        correspondences.push_back(closest_index);
    }
}

Eigen::Matrix3f computeTransform(const std::vector<Eigen::Vector2f> &source, const std::vector<Eigen::Vector2f> &target, const std::vector<int> &correspondences)
{
    int n_points {static_cast<int>(source.size())};
    
    // Variables for least squares solution
    double sumSx = 0.0, sumSy = 0.0, sumTx = 0.0, sumTy = 0.0;
    double sumSxx = 0.0, sumSyy = 0.0, sumSxy = 0.0;
    double sumTxSx = 0.0, sumTxSy = 0.0, sumTySx = 0.0, sumTySy = 0.0;

    for (int i = 0; i < n_points; ++i) {
        const Eigen::Vector2f& s = source[i];
        const Eigen::Vector2f& t = target[correspondences[i]];

        // Accumulate sums for least squares solution
        sumSx += s.x();
        sumSy += s.y();
        sumTx += t.x();
        sumTy += t.y();
        sumSxx += s.x() * s.x();
        sumSyy += s.y() * s.y();
        sumSxy += s.x() * s.y();
        sumTxSx += t.x() * s.x();
        sumTxSy += t.x() * s.y();
        sumTySx += t.y() * s.x();
        sumTySy += t.y() * s.y();
    }

    // Compute transformation parameters
    double N = static_cast<double>(n_points);
    double sx = sumSx / N;
    double sy = sumSy / N;
    double tx = (sumTx - sx * sumTxSx - sy * sumTxSy) / N;
    double ty = (sumTy - sx * sumTySx - sy * sumTySy) / N;

    // Compute rotation angle (assuming scale factor of 1)
    double theta = atan2(sumSxy - sx * sumSy, sumSxx - sx * sx);

    // Compute cosine and sine of rotation angle
    double c = cos(theta);
    double s = sin(theta);

    // Construct the 3x3 transformation matrix
    Eigen::Matrix3f T;
    T << c, -s, tx,
        s, c,  ty,
        0, 0,  1;

    return T;
}

} // namespace icp
