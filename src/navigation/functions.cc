//========================================================================
/*!
\file    functions.cc
\brief   Interface for time optimal controller implementation
\author  Daniel Meza, Jared Rosenbaum, Steven Swanbeck, (C) 2024
*/
//========================================================================

#include "functions.h"


Vector2f Transform_ICOM(float x, float y, float theta, float r){
    Eigen::Vector3f input(x, y, 1);
    Eigen::Matrix3f transformation;
    transformation << cos(theta), -sin(theta), r*sin(theta), sin(theta), cos(theta), r-r*cos(theta), 0, 0, 1;
    Eigen::Vector3f output_3;
    output_3 = transformation*input;
    Eigen::Vector2f output;
    output(0) = output_3(0);
    output(1) = output_3(1);
    return output;
}
