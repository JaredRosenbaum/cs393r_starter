#pragma once

// Correlative Scan Match values
#define ODOM_TRANSLATION_THRESHOLD          1.0     // m
#define ODOM_ROTATION_THRESHOLD             0.35 // 0.52 // 0.6981  // 40 deg

#define MOTION_MODEL_X_RESOLUTION           0.03  //0.5  // m
#define MOTION_MODEL_X_LIMIT                0.15     // m
#define MOTION_MODEL_Y_RESOLUTION           0.03    // m
#define MOTION_MODEL_Y_LIMIT                0.15   //0.5  // m
#define MOTION_MODEL_THETA_RESOLUTION       0.03   // 0.4 // ~2.86deg
#define MOTION_MODEL_THETA_LIMIT            1.2    // ~20deg

#define LOOKUP_TABLE_RESOLUTION             0.03 // m
#define LOOKUP_TABLE_SIGMA                  0.01 // 0.1 // m

#define GTSAM_FREQUENCY                     1 // gtsam will run every this many CSM updates
#define POSE_GRAPH_CONNECTION_DEPTH         5 // number of previous non-sequential poses to connect to new

#define PROBABILITY_WEIGHT                   1 // used to calculate probability for candidate pose, might not actually do anything, but might affect the covariances, which might affect the way the pose graph optimization works; need to test once optimization is implemented

#define K2              0.5 //0.5
#define K3              0.5 //0.1
#define K4              0.1 //0.1
#define K5              0.5 //0.5
#define K6              0.5 //0.5
