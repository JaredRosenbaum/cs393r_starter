#include "path_generation.h"
#include <cstdio>
#include <iostream>
// #include <cmath>
// 1d time optimal control
// given distance to go, max decel, max vel
// out vel

// do this on path option after selecting it

namespace path_generation {

// float run1DTimeOptimalControl(float dist_to_go, float current_speed, const NavigationParams& robot_config) {
//     float max_accel = robot_config.max_accel;
//     float max_decel = robot_config.max_decel;
//     float max_vel = robot_config.max_vel;
//     float dt = robot_config.dt;
//     float cruise_stopping_dist = pow(current_speed, 2) / (2 * max_decel);
//     float accel_stopping_dist = pow(current_speed + dt * max_accel, 2) / (2 * max_decel);
//     // std::cout << "Current Speed: " << current_speed << std::endl;
//     // std::cout << "Max Velocity: " << max_vel << std::endl;
//     // std::cout << "dt: " << dt << std::endl;
//     // std::cout << "Cruise Stopping Distance: " << cruise_stopping_dist << std::endl;
//     // std::cout << "Dist to go: " << dist_to_go << std::endl;

//     // if dist_to_go is larger than stopping_dist and not at max vel, can accelerate
//     if (dist_to_go > accel_stopping_dist && current_speed < max_vel) {
//         return std::min(max_vel, current_speed + max_accel * dt);
//     }
//     else if (dist_to_go > cruise_stopping_dist && current_speed == max_vel) {  // can stop in time and at max vel
//                                                                         // probably needs hysteresis
//         return current_speed;
//     }
//     else {  // otherwise needs to decelerate
//         return std::max(current_speed - max_decel * dt, 0.0f);
//     }
// }



// set curvature, free path length, obstruction for a path option
void setPathOption(Path& path_option,
                        float curvature, const std::vector<Eigen::Vector2f>& point_cloud,
                        const vehicles::Car& robot_config,
                        const Vector2f goal,
                        const Vector2f global_goal) {
    path_option.curvature = curvature;
    float h {(robot_config.dimensions_.length_ + robot_config.dimensions_.wheelbase_) / 2}; // distance from base link to front bumper
    Vector2f projected_pos(0.0, 0.0);
    float goal_distance = goal.norm();
    float global_goal_distance = global_goal.norm();

    if (std::abs(curvature) <= std::abs(0.01)) {
        // fpl
        for (auto p: point_cloud) {
            if (robot_config.dimensions_.width_ / 2 + CAR_MARGIN >= std::abs(p[1])
                && p[0] < path_option.free_path_length) {
                path_option.free_path_length = p[0] - h - CAR_MARGIN;
                path_option.obstruction = p;
            }
        }
        // goal distance
        projected_pos.x() = robot_config.limits_.max_speed_ * TIME_STEP;
        float new_goal_dist = (goal-projected_pos).norm();
        // goal_distance = (goal-projected_pos).norm();
        path_option.dist_to_goal = goal_distance-new_goal_dist;
        // Cap free path length when approaching the goal
        if (global_goal_distance < path_option.free_path_length) {
            path_option.free_path_length = global_goal_distance;
        }
        // clearance
        for (const auto &p: point_cloud) {
            if (p[0] >=0 and p[0] < path_option.free_path_length) {
                float clearance_p = std::abs(p[1]) - robot_config.dimensions_.width_ / 2 - CAR_MARGIN;
                if (clearance_p < path_option.clearance) {
                    path_option.clearance = clearance_p;
                    path_option.closest_point = p;
                }
            }
        }
        return;
    }

    Vector2f c = Vector2f(0, 1 / curvature);
    float r_inner = c.norm() - robot_config.dimensions_.width_ / 2 - CAR_MARGIN;
    float r_outer = c.norm() + robot_config.dimensions_.width_ / 2 + CAR_MARGIN;
    float r_tl = (Vector2f(0, r_inner) - Vector2f(h + CAR_MARGIN, 0)).norm();
    float r_tr = (Vector2f(0, r_outer) - Vector2f(h + CAR_MARGIN, 0)).norm();
    float r_br = (Vector2f(0, r_outer) - Vector2f((robot_config.dimensions_.length_ - robot_config.dimensions_.wheelbase_) / 2 + CAR_MARGIN, 0)).norm();
    path_option.free_path_length = std::min(M_PI * c.norm(), 5.0); //5.0);  // some large number
    // float omega = atan2(h, r_inner);



    float theta_br = asin((robot_config.dimensions_.length_ + robot_config.dimensions_.wheelbase_) / 2 + CAR_MARGIN / r_br); // angle where back right would hit
    float phi = 0;
//	cout << "curvature " << curvature << endl;
//	bool front_side = false, outer_side = false, inner_side = false;
    for (unsigned int i = 0; i < point_cloud.size(); i++) {
        Vector2f p = point_cloud[i];
        float r_p = (c-p).norm();
        float theta = curvature < 0 ? atan2(p[0], p[1]- c[1]) : atan2(p[0], c[1] - p[1]); // angle between p and c
        float length = 5.0; //5.0;
        // cout << "curvature " << curvature << endl;
        if (r_inner <= r_p && r_p <= r_tl) {    // inner side hit
                phi = acos(r_inner / r_p);
                length = (theta - phi) * c.norm();
            // inner_side = true;
                // cout << "inner side hit" << endl;
        }
        if ((r_inner <= r_p && r_p <= r_br) && (-theta_br <= theta && theta <= theta_br)) {    // outer side hit
            phi = acos(r_p / (c.norm() + robot_config.dimensions_.width_ / 2));
            length = (theta - phi) * c.norm();
	    // outer_side = true;
	    // cout << "outer side hit" << endl;
        }

        if (r_tl <= r_p && r_p <= r_tr) {    // front side hit
            phi = asin(h / r_p);
            length = (theta - phi) * c.norm();
	    // front_side = true;
	    // cout << "front side hit" << endl;
        }
        if (length < path_option.free_path_length && length > 0) {
            path_option.free_path_length = length;
            path_option.obstruction = p;
        }
    }
	// if (inner_side)
	//  	cout << "intersecting particle found with inner side" << endl;
	// if (outer_side)
	//	cout << "intersecting particle found with outer side" << endl;
	//if (front_side)
	//	cout << "intersecting particle found with front side" << endl;

    // float theta = M_PI / 2;
    // if (path_option.obstruction != Eigen::Vector2f::Zero()) {
    //     theta = curvature < 0 ? atan2(path_option.obstruction[0], path_option.obstruction[1]- c[1]) :
    //         atan2(path_option.obstruction[0], c[1] - path_option.obstruction[1]);
    // }
    //. Distance to goal
    float radius {1.0f / curvature};
    float phi_g = (robot_config.limits_.max_speed_ * TIME_STEP * 20) / radius;
    projected_pos.x() = radius * sin(phi_g);
    projected_pos.y() = radius - (radius * cos(phi_g));
    float new_goal_dist = (goal-projected_pos).norm();
    // goal_distance = (goal-projected_pos).norm();
    path_option.dist_to_goal = goal_distance-new_goal_dist;
    // Cap free path length when approaching the goal
    if (global_goal_distance < path_option.free_path_length) {
        path_option.free_path_length = global_goal_distance;
    }
    // clearance
    // path_option.clearance = 100; // some large number
    for (auto p: point_cloud) {
        float theta_p =  curvature < 0 ? atan2(p[0], p[1]- c[1]) :
            atan2(p[0], c[1] - p[1]);
        float path_len_p = theta_p * (p-c).norm();
        if (path_len_p >=0 and path_len_p < path_option.free_path_length) {  // if p is within the fp length
            float inner = std::abs((c - p).norm() - r_inner);
            float outer = std::abs((c - p).norm() - r_tr);
            float clearance_p = std::min(inner, outer);
            if (clearance_p < path_option.clearance) {
                path_option.clearance = clearance_p;
                path_option.closest_point = p;
            }
        }
    }

}


// sample path options
// given point cloud (robot frame), num options, max curvature
// out const std::vector path options

std::vector<Path> samplePathOptions(int num_options,
                                                    const std::vector<Eigen::Vector2f>& point_cloud,
                                                    const vehicles::Car &robot_config,
                                                    const Vector2f goal,
                                                    const Vector2f global_goal) {
    static std::vector<Path> path_options;
    path_options.clear();
    float max_curvature = robot_config.limits_.max_curvature_;

    // loop through curvature from max to -max
    for (int i = 0; i < num_options; i++) { 
        float curvature = max_curvature * pow(2*i/float(num_options-1) - 1, 2);
        if (i < num_options / 2) {
            curvature = -curvature;
        }
        
        Path path_option;
        setPathOption(path_option, curvature, point_cloud, robot_config, goal, global_goal);
        path_options.push_back(path_option);
    }
    // exit(0);
    return path_options;
}


float score(float free_path_length, float curvature, float clearance, float goal_dist) {
    const float w1 = 0.75; //1.0;
    const float w2 = 0;
    const float w3 = 0.3; //0.1;
    const float w4 = 5.0; //15;
    return w1 * free_path_length + w2 * std::abs(1/curvature) + w3 * clearance + w4 * goal_dist;
}

// returns the index of the selected path
// for now, just return the index of the path with the longest free path length
// if there are multiple paths with the same free path length, return the one with the smallest curvature
int selectPath(const std::vector<Path>& path_options) {
    int selected_path = 0;
    float best_score = 0;
    for (unsigned int i = 0; i < path_options.size(); i++) {
        float s = score(path_options[i].free_path_length, path_options[i].curvature, path_options[i].clearance, path_options[i].dist_to_goal);
        if (s > best_score) {
            best_score = s;
            selected_path = i;
        }
    }
    return selected_path;
}

} // namespace path_generation
