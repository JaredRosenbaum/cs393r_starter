
#ifndef PATH_GENERATION_H
#define PATH_GENERATION_H

#include <vector>
#include "eigen3/Eigen/Dense"
// #include "parameters.h"
#include "vehicles.hpp"

using std::vector;
using Eigen::Vector2f;

namespace path_generation {

struct Path {
  float curvature = 0;
  float clearance = 10;
  float free_path_length = 10;
  float dist_to_goal = 20;
  Eigen::Vector2f obstruction = Eigen::Vector2f::Zero();
  Eigen::Vector2f closest_point = Eigen::Vector2f::Zero();
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

// float run1DTimeOptimalControl(float dist_to_go, float current_speed, const NavigationParams& nav_params);

void setPathOption(Path& path_option,
    float curvature,
    const std::vector<Eigen::Vector2f>& point_cloud,
    const vehicles::Car& robot_config,
    const Vector2f goal);

std::vector<Path> samplePathOptions(int num_options,
                                                    const std::vector<Eigen::Vector2f>& point_cloud,
                                                    const vehicles::Car& robot_config,
                                                    const Vector2f goal);

int selectPath(const std::vector<Path>& path_options);

} // namespace path_generation

#endif // PATH_GENERATION_H
