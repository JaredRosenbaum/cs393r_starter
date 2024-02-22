//========================================================================
/*!
\file    functions.h
\brief   Useful robotic functions
\author  Daniel Meza, Jared Rosenbaum, Steven Swanbeck, (C) 2024
*/
//========================================================================

#pragma once

#include <iostream>
#include <cmath>
#include <vector>

#include "eigen3/Eigen/Dense"

namespace utils {

namespace transforms {

Eigen::Vector2f transformICOM(float x, float y, float theta, float r);

Eigen::Vector2f projectPoint(Eigen::Vector2f point, float theta, float radius);

} // namespace transforms

namespace testing {

std::vector<Eigen::Vector2f> generateTestCloud();

} // namespace testing

} // namespace utils
