#pragma once

#define LATENCY                         0.00 //0.10    // s
#define TIME_STEP                       0.05    // s
#define MAX_SPEED                       1.0     // m/s
#define MAX_ACCELERATION                4.0     // m/s^2
#define MAX_CURVATURE                   1.0     // m^(-1)
#define CURVATURE_SAMPLING_INTERVAL     0.05   // m
#define CAR_WIDTH                       0.281   // m
#define CAR_LENGTH                      0.535   // m
#define CAR_WHEELBASE                   0.324   // m
#define CAR_TRACK_WIDTH                 0.227   // m
#define CAR_MARGIN                      0.1 //0.15    // m
#define MAX_CLEARANCE                   0.5 //0.5     // m

struct NavigationParams {
  // frequency
  float dt = .05f;
  // max velocity
  float max_vel = 1.0f;
  // max acceleration
  float max_accel = 4.0f;
  float max_decel = 4.0f;
  // max angular velocity
  float max_omega = 1.0f;
  // max angular acceleration
  float max_alpha = 1.0f;
  // max curvature
  float max_curvature = 1.0f;
  // safety margin
  float safety_margin = 0.1f;

  // robot dimensions
  float  width = 0.281f;
  float  length = 0.535f;
  float  wheelbase = 0.324f;
  float  base_link_offset = 0.106f; // make this 0 for now

  // delays
  float actuation_latency = 0.0f;
  float observation_latency = 0.0f;
};
