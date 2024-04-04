#pragma once

// Obstacle avoidance values
#define LATENCY                         0.1 //0.10    // s
#define TIME_STEP                       0.05    // s
#define MAX_SPEED                       1.0     // m/s
#define MAX_ACCELERATION                4.0     // m/s^2
#define MAX_CURVATURE                   1.0     // m^(-1)
#define CURVATURE_SAMPLING_INTERVAL     0.05   // m
#define N_PATHS                         31
// Hardware values
#define CAR_WIDTH                       0.281   // m
#define CAR_LENGTH                      0.535   // m
#define CAR_WHEELBASE                   0.324   // m
#define CAR_TRACK_WIDTH                 0.227   // m
#define CAR_MARGIN                      0.1 //0.15    // m
#define MAX_CLEARANCE                   0.5 //0.5     // m
// Carrot planner values
#define STICK_LENGTH                    10.0 // 5m
#define GOAL_TOL                        0.3 // 0.25m
#define PATH_DEV_TOL                    3.0 //1m
// RRT* Global Planner values
#define GOAL_THRESHOLD                  0.375
#define GRAPH_RESOLUTION                0.2
#define COLLISION_PROXIMITY             (CAR_WIDTH / 2) + CAR_MARGIN
#define SAMPLE_BUFFER                   5.0
#define OPTIMIZATION_RADIUS             5.0
#define MAX_SAMPLING_ITERATIONS         500000
