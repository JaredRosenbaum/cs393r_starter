#pragma once

// Correlative Scan Match values
#define ODOM_TRANSLATION_THRESHOLD          0.5     // m
#define ODOM_ROTATION_THRESHOLD             0.6981  // 40 deg
// #define MOTION_MODEL_X_RESOLUTION           0.05    // m
// #define MOTION_MODEL_X_LIMIT                0.05     // m
// #define MOTION_MODEL_Y_RESOLUTION           0.05    // m
// #define MOTION_MODEL_Y_LIMIT                0.5     // m
// #define MOTION_MODEL_THETA_RESOLUTION       0.05    // ~2.86deg
// #define MOTION_MODEL_THETA_LIMIT            0.35    // ~20deg
#define MOTION_MODEL_X_RESOLUTION           0.5    // m
#define MOTION_MODEL_X_LIMIT                1.0     // m
#define MOTION_MODEL_Y_RESOLUTION           0.5    // m
#define MOTION_MODEL_Y_LIMIT                1.0     // m
#define MOTION_MODEL_THETA_RESOLUTION       0.4    // ~2.86deg
#define MOTION_MODEL_THETA_LIMIT            0.4    // ~20deg
