#pragma once

#include <iostream>
#include <vector>
#include <memory>
#include <eigen3/Eigen/Dense>

namespace motion_model {

class MultivariateMotionModel
{
public:
    MultivariateMotionModel(const Eigen::Vector3d &odom)
    {
        // storing values and precomputing ones needed for computation
        _mu = std::make_unique<const Eigen::Vector3d>(odom);
        
        Eigen::Vector2d translation_vector(odom.x(), odom.y());
        const double translation {translation_vector.norm()};
        const double rotation {std::abs(odom.z())};
        const double k2 {0.5d};
        const double k3 {0.1d};
        const double k4 {0.1d};
        const double k5 {0.5d};
        const double k6 {0.5d};

        Eigen::Matrix3d cov;
        cov << std::max(pow(k5 * translation + k4 * rotation, 2), 0.1), 0.d, 0.d,
                0.d, std::max(pow(k6 * translation + k4 * rotation, 2), 0.1), 0.d,
                0.d, 0.d, std::max(pow(k2 * translation + k3 * rotation, 2), 0.1);

        _covariance = std::make_unique<const Eigen::Matrix3d>(cov);

        // use LU decomposition to get the inverse and determinant (precomputing so we don't have to do it for each evaluation)
        Eigen::FullPivLU<Eigen::Matrix3d> lu_decomp(cov);

        if (lu_decomp.rank() == 3) {
            _inv_covariance = std::make_unique<const Eigen::Matrix3d>(lu_decomp.inverse());

            // _det_covariance = std::make_unique<const double>(lu_decomp.determinant());
            // _normalization_constant = std::make_unique<const double>(
            //     1 / (sqrt(pow(2 * M_PI, 3) * (*_det_covariance)))
            // );
        }

        // std::cout << *_covariance << ",\n" << *_inv_covariance << ",\n" << *_det_covariance << ",\n" << *_normalization_constant << std::endl;
    }

    double evaluate(const Eigen::Vector3d &X)
    {
        if (_inv_covariance == nullptr) {
            std::cerr << "Inverse covariance or determinant could not be set! Covariance was probably not positive definite. Returning a small non-zero probability to allow computation to continue." << std::endl;
            return 0.1;
        }

        // return (*_normalization_constant) * exp(-0.5 * (X - (*_mu)).transpose() * (*_inv_covariance) * (X - (*_mu)));
        
        // just returning unnormalized probability because we only care about proportionality
        return exp(-0.5 * (X - (*_mu)).transpose() * (*_inv_covariance) * (X - (*_mu)));
    }

private:
    std::unique_ptr<const Eigen::Vector3d> _mu;
    std::unique_ptr<const Eigen::Matrix3d> _covariance;
    std::unique_ptr<const Eigen::Matrix3d> _inv_covariance;
    // std::unique_ptr<const double> _det_covariance;
    // std::unique_ptr<const double> _normalization_constant; 

}; // class MultivariateMotionModel

class IndependentMotionModel
{
public:
    IndependentMotionModel(const Eigen::Vector3d &odom)
    : _x(odom.x()), _y(odom.y()), _theta(odom.z())
    {
        // storing inputs

        // compute variances
        const double k2 {0.5d};
        const double k3 {0.1d};
        const double k4 {0.1d};
        const double k5 {0.5d};
        const double k6 {0.5d};

        _x_std = k5 * sqrt(pow(_x, 2) + pow(_y, 2)) + k4 * _theta;
        _y_std = k6 * sqrt(pow(_x, 2) + pow(_y, 2)) + k4 * _theta;
        _theta_std = k2 * sqrt(pow(_x, 2) + pow(_y, 2)) + k3 * _theta;

        // now compute normalization constants
        _x_norm = 1.d / (_x_std * sqrt(2 * M_PI));
        _y_norm = 1.d / (_y_std * sqrt(2 * M_PI));
        _theta_norm = 1.d / (_theta_std * sqrt(2 * M_PI));
    }

    double evaluate(const Eigen::Vector3d &X)
    {
        double p_x {_x_norm * exp(-0.5 * pow((X.x() - _x) / _x_std, 2))};
        double p_y {_y_norm * exp(-0.5 * pow((X.y() - _y) / _y_std, 2))};
        double p_theta {_theta_norm * exp(-0.5 * pow((X.z() - _theta) / _theta_std, 2))};

        std::cout << p_x << ", " << p_y << ", " << p_theta << ", " << p_x * p_y * p_theta << std::endl;

        return p_x * p_y * p_theta;
    }

private:
    const double _x;
    const double _y;
    const double _theta;

    double _x_std; 
    double _y_std; 
    double _theta_std;
    
    double _x_norm;
    double _y_norm;
    double _theta_norm;

}; // class IndependentMotionModel

} // namespace motion_model
