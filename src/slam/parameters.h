#pragma once

// Correlative Scan Match values
#define ODOM_TRANSLATION_THRESHOLD          1.0     // m
#define ODOM_ROTATION_THRESHOLD             0.52    // rad

// Generate Candidate values
#define POSE_SAMPLING_X_RESOLUTION           0.03    // m
#define POSE_SAMPLING_X_LIMIT                0.15    // m
#define POSE_SAMPLING_Y_RESOLUTION           0.03    // m
#define POSE_SAMPLING_Y_LIMIT                0.15    // m
#define POSE_SAMPLING_THETA_RESOLUTION       0.03    // ~1.71 deg
#define POSE_SAMPLING_THETA_LIMIT            0.4     // ~17.19 deg

#define LOOKUP_TABLE_RESOLUTION             0.03 // m
#define LOOKUP_TABLE_SIGMA                  0.03 // 0.05 // 0.01 // 0.1 // m

#define GTSAM_FREQUENCY                     1 // gtsam will run every this many CSM updates
#define POSE_GRAPH_CONNECTION_DEPTH         2 // 5 // number of previous non-sequential poses to connect to new

#define K2              0.5
#define K3              0.5
#define K4              0.1
#define K5              0.5
#define K6              0.5
