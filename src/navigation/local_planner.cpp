//========================================================================
/*!
\file    local_planner.cpp
\brief   todo
\author  Daniel Meza, Jared Rosenbaum, Steven Swanbeck, (C) 2024
*/
//========================================================================

#include "local_planner.h"





namespace local_planners {

// -------------------------------------------------------------------------
// * CARROT PLANNER
// -------------------------------------------------------------------------

CarrotPlanner::CarrotPlanner(float stick_length, float goal_tolerance, float deviation_tolerance)
: stick_length_(stick_length), goal_tolerance_(goal_tolerance), deviation_tolerance_(deviation_tolerance)
{}

void CarrotPlanner::populatePath(std::vector<Vector2f> path){
    full_path_ = path;
    return;
}

bool CarrotPlanner::reachedGoal(Vector2f robot_xy, Vector2f goal_xy){
    // Calculate the distance from the robot to the goal
    Vector2f to_goal = goal_xy - robot_xy;
    float goal_dist = to_goal.norm();
    // Decide if the robot should continue to move
    if (goal_dist > goal_tolerance_){
        return false;
    }
    else{
        ROS_INFO_STREAM("Robot has reached the goal. No local planning will occur.");
        return true;
    }
}

bool CarrotPlanner::planStillValid(Vector2f robot_xy){
    // Is the robot within a tolerance of any point on the path?
    bool near_path = false;
    for (int i = full_path_.size()-1; i >= 0; i--){
        Vector2f to_edge = full_path_[i] - robot_xy;
        float edge_dist = to_edge.norm();
        if (edge_dist <= deviation_tolerance_){
            near_path = true;
        }
    }
    // If yes, continue to plan!
    if (near_path){
        return true;
    }
    // If no, replanning is required.
    else{
        ROS_INFO_STREAM("Robot has strayed too far from the path. Replanning is required.");
        return false;
    }
}

Vector2f CarrotPlanner::feedCarrot(Vector2f robot_xy){
    int best_index = -1;
    // Work backwards from the goal to the start
    for (int i = full_path_.size()-1; i >= 0; i--){
        Vector2f to_edge = full_path_[i] - robot_xy;
        float edge_dist = to_edge.norm();
        // Once an edge is found that is within the carrot planners allowed radius, make that the carrot.
        if (edge_dist <= stick_length_){
            best_index = i;
            break;
        }
    }
    if (best_index < 0){
        // No edges fell within the carrot planners radius. This should NEVER occur, thanks to the planStillValid function. If it occurs, something bad happened!
        ROS_INFO_STREAM("Carrot planner failed.");
        return robot_xy;
    }
    else{
        return full_path_[best_index];
    }
}

}
