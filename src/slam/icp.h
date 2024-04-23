#pragma once

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

namespace icp {

bool performICP(const std::vector<Eigen::Vector2f> &source, const std::vector<Eigen::Vector2f> &target, const int max_iterations, const double tolerance);

Eigen::Vector2f transformPoint(const Eigen::Vector2f &point, const Eigen::Matrix3f &T);

std::vector<Eigen::Vector2f> transformPoints(const std::vector<Eigen::Vector2f> &points, const Eigen::Matrix3f &T);

void findCorrespondences(const std::vector<Eigen::Vector2f> &source, const std::vector<Eigen::Vector2f> &target, std::vector<int> &correspondences);

Eigen::Matrix3f computeTransform(const std::vector<Eigen::Vector2f> &source, const std::vector<Eigen::Vector2f> &target, const std::vector<int> &correspondences);

} // namespace icp
