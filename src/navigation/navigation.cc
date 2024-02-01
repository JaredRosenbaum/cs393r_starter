//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

#define TIME_STEP         0.05
#define MAX_SPEED         1.0
#define MAX_ACCELERATION  4.0
#define MAX_CURVATURE     1.0
#define CURVATURE_STEP    0.05
#define CAR_WIDTH         0.265   // TODO Double check number
#define CAR_LENGTH        0.5     // TODO Update with correct value
#define CAR_WHEELBASE     0.345
#define CAR_TRACK_WIDTH   0.227
#define CAR_MARGIN        0.05

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;

// Time Optimal Controller variables
float distance_traveled_ = 0;
float goal_distance_ = 10;
float control_speed_;
float control_curvature_;

} //namespace

namespace navigation {

string GetMapFileFromName(const string& map) {
  string maps_dir_ = ros::package::getPath("amrl_maps");
  return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
}

Navigation::Navigation(const string& map_name, ros::NodeHandle* n) :
  // #
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0) {
  map_.Load(GetMapFileFromName(map_name));
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);

  // + 
  // controller_ = new LatencyController();
  controller_ = new TimeOptimalController(TIME_STEP, MAX_SPEED, MAX_ACCELERATION, MAX_CURVATURE, CURVATURE_STEP, CAR_WIDTH, CAR_LENGTH, CAR_WHEELBASE, CAR_TRACK_WIDTH, CAR_MARGIN);
  // +
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    odom_loc_ = loc;
    odom_angle_ = angle;
    return;
  }
  odom_loc_ = loc;
  odom_angle_ = angle;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;                                     
}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Print debug statement
  // std::cout << "x: " << distance_traveled_ << std::endl;
  // std::cout << "odom:" << sqrt(pow(odom_loc_[0] - odom_start_loc_[0], 2) + pow(odom_loc_[1] - odom_start_loc_[1], 2)) << std::endl;

  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // // Visualize pointcloud
  // for (int i = 0; i < (int)point_cloud_.size(); i++) {
  //   Vector2f p = point_cloud_[i];
  //   visualization::DrawCross(p, 0.01, 0, local_viz_msg_);
  // }

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // Note: curvature not theta for Calc FPL
  controller_->CalculateFreePathLength(point_cloud_, 0.0);

  // Run the time optimal controller to calculate drive commands
  float distance_left = goal_distance_ - distance_traveled_;
  control_speed_ = controller_->CalculateSpeed(distance_left);
  distance_traveled_ += control_speed_ * TIME_STEP;

  // The control iteration goes here.

  // The latest observed point cloud is accessible via "point_cloud_"

  // Eventually, you will have to set the control values to issue drive commands:
  drive_msg_.curvature = control_curvature_;
  drive_msg_.velocity = control_speed_;

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();

  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
}

}  // namespace navigation
