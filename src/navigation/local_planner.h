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
#include "shared/math/line2d.h"

#include "ros/ros.h"
#include "vector_map/vector_map.h"


#include "vehicles.hpp"
#include "functions.h"

using Eigen::Vector2f;
using geometry::line2f;


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
        bool init = false;
        float stick_length_; // Radius around car to check for intersection with path
        float goal_tolerance_; // How close (radially) to the goal must the car be to stop planning?
        float deviation_tolerance_; // How close to the path must the robot be for it to not require a replanning? DEVIATION TOLERANCE MUST BE LESS THAN STICK LENGTH


};

//. Other local planners can go here
class SmoothedPlanner{
    public:
        SmoothedPlanner(const vector_map::VectorMap map, float stick_length, float goal_tolerance, float deviation_tolerance);

        void populatePath(std::vector<Vector2f> path);

        bool reachedGoal(Vector2f robot_xy, Vector2f goal_xy);

        bool planStillValid(Vector2f robot_xy);

        bool checkMapCollision(const Eigen::Vector2f point1, const Eigen::Vector2f point2, geometry::line2f &l1, geometry::line2f &l2);

        Vector2f interpolatePath(Vector2f robot_xy, float robot_angle, float interpolation_threshold, geometry::line2f &l1, geometry::line2f &l2);
        // Vector2f feedCarrot(Vector2f robot_xy);
    private:
        std::vector<Vector2f> full_path_;
        bool init = false;
        vector_map::VectorMap map_;   // Map of the environment
        float stick_length_; // Radius around car to check for intersection with path
        float goal_tolerance_; // How close (radially) to the goal must the car be to stop planning?
        float deviation_tolerance_; // How close to the path must the robot be for it to not require a replanning? DEVIATION TOLERANCE MUST BE LESS THAN STICK LENGTH

};


} // namespace local_planners
