#pragma once

#include "parameters.h"

namespace vehicles {

struct Limits {
    const float max_speed_; // m/s
    const float max_acceleration_; // m/s^2
    const float max_curvature_; // m^(-1)

    Limits(float max_speed, float max_acceleration, float max_curvature) : max_speed_(max_speed), max_acceleration_(max_acceleration), max_curvature_(max_curvature) {}
}; // struct Limits

struct Dimensions {
    const float length_; // m
    const float width_; // m
    const float wheelbase_; // m
    const float track_width_; // m

    Dimensions(float length, float width, float wheelbase, float track_width) : length_(length), width_(width), wheelbase_(wheelbase), track_width_(track_width) {}
}; // struct Dimensions

struct Car {
    const Dimensions dimensions_;
    const Limits limits_;

    Car(float length, float width, float wheelbase, float track_width, float max_speed, float max_acceleration, float max_curvature) :
    dimensions_(length, width, wheelbase, track_width), 
    limits_(max_speed, max_acceleration, max_curvature) {}
}; // Struct Car

struct UT_Automata : public Car {
    UT_Automata() : Car(CAR_LENGTH, CAR_WIDTH, CAR_WHEELBASE, CAR_TRACK_WIDTH, MAX_SPEED, MAX_ACCELERATION, MAX_CURVATURE) {}
}; // struct UT_Automata

} // namespace vehicles
