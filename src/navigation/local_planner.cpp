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
    init = true;
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
        std::cout << "[LocalPlanner] Robot has reached the goal. No local planning will occur." << std::endl;
        return true;
    }
}

bool CarrotPlanner::planStillValid(Vector2f robot_xy){
    // Is the robot within a tolerance of any point on the path?
    if (!init) {
        std::cout << "[LocalPlanner] No path!" << std::endl;
        return false;
    }
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
        std::cout << "[LocalPlanner] Robot has strayed too far from the path. Replanning is required." << std::endl;
        return false;
    }
}

Vector2f CarrotPlanner::feedCarrot(Vector2f robot_xy){
    if (!init) {
        std::cout << "[LocalPlanner] No path!" << std::endl;
        return robot_xy;
    }
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
        std::cout << "[LocalPlanner] Carrot planner failed." << std::endl;
        return robot_xy;
    }
    else{
        return full_path_[best_index];
    }
}

// -------------------------------------------------------------------------
// * INTERPOLATING PLANNER
// -------------------------------------------------------------------------
SmoothedPlanner::SmoothedPlanner(const vector_map::VectorMap map, float stick_length, float goal_tolerance, float deviation_tolerance)
: map_(map), stick_length_(stick_length), goal_tolerance_(goal_tolerance), deviation_tolerance_(deviation_tolerance)
{}

void SmoothedPlanner::populatePath(std::vector<Vector2f> path){
    full_path_ = path;
    init = true;
    return;
}

bool SmoothedPlanner::reachedGoal(Vector2f robot_xy, Vector2f goal_xy){
    if (!init) {
        std::cout << "[LocalPlanner] No path!" << std::endl;
        return false;
    }

    //. Calculate the distance from the robot to the goal
    Vector2f to_goal = full_path_[full_path_.size()-1] - robot_xy;
    float goal_dist = to_goal.norm();
    //. Decide if the robot should continue to move
    if (goal_dist > goal_tolerance_){
        return false;
    }
    else{
        std::cout << "[LocalPlanner] Robot has reached the goal. No local planning will occur." << std::endl;
        return true;
    }
}

bool SmoothedPlanner::planStillValid(Vector2f robot_xy){
    if (!init) {
        std::cout << "[LocalPlanner] No path!" << std::endl;
        return false;
    }
    //. Is the robot within a tolerance of any point on the path?
    bool near_path = false;
    for (int i = full_path_.size()-1; i > 0; i--){
        Vector2f a = full_path_[i];
        Vector2f b = full_path_[i-1];
        float a_b_norm = (a-b).norm();
        // std::cout << i << std::endl << std::endl;
        for (float j = 0; j<= a_b_norm; j+=0.25){
            // Get point
            Vector2f point = a-((a-b)*(j/a_b_norm));
            Vector2f to_edge = point - robot_xy;
            float edge_dist = to_edge.norm();
            if (edge_dist <= deviation_tolerance_){
                near_path = true;
            }
        }
    }

    // for (int i = full_path_.size()-1; i >= 0; i--){
    //     Vector2f to_edge = full_path_[i] - robot_xy;
    //     float edge_dist = to_edge.norm();
    //     if (edge_dist <= deviation_tolerance_){
    //         near_path = true;
    //     }
    // }

    //. If yes, continue to plan!
    if (near_path){
        return true;
    }
    //. If no, replanning is required.
    else {
        std::cout << "[LocalPlanner] Robot has strayed too far from the path. Replanning is required." << std::endl;
        return false;
    }
}

bool SmoothedPlanner::checkMapCollision(const Eigen::Vector2f point1, const Eigen::Vector2f point2, geometry::line2f &l1, geometry::line2f &l2) {
    //. Create line between node and its parent.
    geometry::line2f line(point1[0], point1[1],
                point2[0], point2[1]);
    // Note: These seven lines allow for collision checks to have an added margin. Can be added later, but shouldn't be necessary
    float collision_check_width_ = CAR_WIDTH + 2 * CAR_MARGIN; // 0.24f; // 0.15;
    float theta = atan2((point2[1] - point1[1]), (point2[0] - point1[0]));
    // float dx = collision_check_width_ / 2 * cos(M_PI / 2 - theta);
    // float dy = collision_check_width_ / 2 * sin(M_PI / 2 - theta);
    float dx {collision_check_width_ / 2 * sin(theta)};
    float dy {collision_check_width_ / 2 * cos(theta)};
    line2f line1(point1[0] - dx, point1[1] + dy,
                point2[0] - dx, point2[1] + dy);
    line2f line2(point1[0] + dx, point1[1] - dy,
                point2[0] + dx, point2[1] - dy);
    //. Loop through map lines checking for instersections
    bool collision_flag = false;
    for (size_t j = 0; j < map_.lines.size(); ++j) {
        const line2f map_line = map_.lines[j];
        //. Check for instersection
        Eigen::Vector2f intersection_point;
        bool intersects = map_line.Intersection(line, &intersection_point);
        bool intersects1 = map_line.Intersection(line1, &intersection_point);
        bool intersects2 = map_line.Intersection(line2, &intersection_point);

    //. Intersection found
        if (intersects || intersects1 || intersects2) {
            collision_flag = true;
            l1 = line1;
            l2 = line2;
            break;
        }
    }
    return collision_flag;
}

Vector2f SmoothedPlanner::interpolatePath(Vector2f robot_xy, float robot_angle, float interpolation_threshold, geometry::line2f &l1, geometry::line2f &l2){
    Vector2f goal = robot_xy;
    if (!init) {
        std::cout << "[LocalPlanner] No path!" << std::endl;
        return goal;
    }
    for (int i = full_path_.size()-1; i > 0; i--){
        Vector2f a = full_path_[i];
        Vector2f b = full_path_[i-1];
        float a_b_norm = (a-b).norm();
        // std::cout << i << std::endl << std::endl;
        for (float j = 0; j<= a_b_norm; j+=interpolation_threshold){
            // Get point
            Vector2f point = a-((a-b)*(j/a_b_norm));
            // std::cout << point.transpose() << std::endl;
            // Check for intersection, if intersection free make goal
            if ((point-robot_xy).norm() <= stick_length_) {
                if (checkMapCollision(robot_xy, point, l1, l2) == false) {
                    // std::cout << "index: " << i << " far point: " << a.transpose() << " near point: " << b.transpose() << " interpolated point: " << point.transpose() << std::endl;
                    // push the carrot forward a bit if it would be stuck inside the robot (helps get around tight corners)
                    // bool pushed {false};
                    if ((i != static_cast<int>(full_path_.size()) - 1) && (point-robot_xy).norm() <= 0.5) {
                        point.x() += 0.5 * cos(robot_angle);
                        point.y() += 0.5 * sin(robot_angle);
                        // pushed = true;
                    }
                    // std::cout << "i: " << i << ", path_size: " << static_cast<int>(full_path_.size()) - 1 << ", point: " << point.transpose() << ", pushed: " << pushed << std::endl;
                    return point;
                }
            }
        }
    }
    // if we made it this far, let's push the goal to be a bit in front of the robot's location to keep it moving
    // goal.x() += 0.15 * cos(robot_angle);
    // goal.y() += 0.15 * sin(robot_angle);
    std::cout << "goal: " << goal.transpose() << std::endl;
    return goal;
}
}
