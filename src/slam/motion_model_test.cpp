#include "motion_model.hpp"

#include <iostream>
#include <eigen3/Eigen/Dense>

int main (int argc, char **argv)
{
    Eigen::Vector3d odom(0.5, 0, 30 * M_PI / 180);
    Eigen::Vector3d eval(0.47, -0.04, 27 * M_PI / 180);

    // auto independent_model {motion_model::IndependentMotionModel(odom)};
    // std::cout << independent_model.evaluate(eval) << std::endl;
    
    auto model {motion_model::MultivariateMotionModel(odom, odom)};
    std::cout << model.evaluate(eval) << std::endl;
    
    return 0;
}
