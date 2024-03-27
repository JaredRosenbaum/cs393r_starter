
#ifndef PATH_OPTIONS_H
#define PATH_OPTIONS_H

#include <vector>
#include "eigen3/Eigen/Dense"
#include "parameters.h"

using std::vector;
using Eigen::Vector2f;

namespace path_options {

struct PathOption {
  float curvature = 0;
  float clearance = 10;
  float free_path_length = 10;
  Eigen::Vector2f obstruction = Eigen::Vector2f::Zero();
  Eigen::Vector2f closest_point = Eigen::Vector2f::Zero();
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

float run1DTimeOptimalControl(float dist_to_go, float current_speed, const NavigationParams& nav_params);

void setPathOption(PathOption& path_option,
    float curvature,
    const std::vector<Eigen::Vector2f>& point_cloud,
    const NavigationParams& nav_params);

std::vector<PathOption> samplePathOptions(int num_options,
                                                    const std::vector<Eigen::Vector2f>& point_cloud,
                                                    const NavigationParams& robot_config);

int selectPath(const std::vector<PathOption>& path_options);

} // namespace path_options

#endif // PATH_OPTIONS_H
