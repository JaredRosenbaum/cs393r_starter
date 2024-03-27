//========================================================================
/*!
\file    local_planner.h
\brief   todo
\author  Daniel Meza, Jared Rosenbaum, Steven Swanbeck, (C) 2024
*/
//========================================================================


#pragma once

#include <iostream>
#include <cmath>
#include <vector>
#include "eigen3/Eigen/Dense"
#include "ros/ros.h"

#include "vehicles.hpp"
#include "functions.h"

#include "amrl_msgs/VisualizationMsg.h"
using amrl_msgs::VisualizationMsg;

using Eigen::Vector2f;

namespace local_planners {

class CarrotPlanner{
    public:
        CarrotPlanner(float stick_length, float goal_tolerance, float deviation_tolerance);

        void populatePath(std::vector<Vector2f> path);

        bool reachedGoal(Vector2f robot_xy, Vector2f goal_xy);

        bool planStillValid(Vector2f robot_xy);

        Vector2f feedCarrot(Vector2f robot_xy);

        // void drawPath(VisualizationMsg msg);


    private:
        std::vector<Vector2f> full_path_;
        float stick_length_; // Radius around car to check for intersection with path
        float goal_tolerance_; // How close (radially) to the goal must the car be to stop planning?
        float deviation_tolerance_; // How close to the path must the robot be for it to not require a replanning? DEVIATION TOLERANCE MUST BE LESS THAN STICK LENGTH


};

//. Other local planners can go here


} // namespace local_planners
