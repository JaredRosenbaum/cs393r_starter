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

// +
#include "parameters.h"
// +


namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;

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
    nav_goal_angle_(0)
    {
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
  // instantiating the car
  car_ = new vehicles::UT_Automata(); // - note this inherits from vehicles::Car and uses the parameters defined in parameters.h

  // instantiating the controller
  controller_ = new controllers::time_optimal_1D::Controller(car_, TIME_STEP, CAR_MARGIN, MAX_CLEARANCE, CURVATURE_SAMPLING_INTERVAL);

  latency_controller_ = new controllers::latency_compensation::Controller(car_, TIME_STEP, CAR_MARGIN, MAX_CLEARANCE, CURVATURE_SAMPLING_INTERVAL, LATENCY);
  // +

  // instantiate a global planner
  global_planner_ = new global_planner::Global_Planner(map_, n);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
  // Update navigation goal
  nav_goal_loc_ = loc;
  nav_goal_angle_ = angle;

  std::cout << "[Navigation] Calculating global path from (" << robot_loc_[0] << ", " << robot_loc_[1] << ") to (" << loc[0] << ", " << loc[1] << ")" << std::endl;

  // Clear previous global path
  global_path_found_ = false;
  global_planner_->ClearPath();

  // Calculate global path from planner
  global_planner_->SetRobotLocation(robot_loc_);
  global_planner_->SetGoalLocation(nav_goal_loc_);
  global_path_found_ = global_planner_->CalculatePath(25000);

  if (global_path_found_) {
    std::cout << "[Navigation] Global path ready!" << std::endl;
  }
  else{
    std::cout << "[Navigation] Global path not found!" << std::endl;
  }
  nav_complete_ = false;
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
  last_msg_timestamp_ = time;                       
}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.

  // TODO Remove
  return;

  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);


  // Visualize pointcloud
  // for (int i = 0; i < (int)point_cloud_.size(); i++) {
  //   Vector2f p = point_cloud_[i];
  //   visualization::DrawCross(p, 0.01, 0, local_viz_msg_);
  // }

  // Visualize car corners
  Vector2f p(0.0, 0.0);
  visualization::DrawCross(p, 0.01, 0, local_viz_msg_); // base_link
  p.x() = -(car_->dimensions_.length_ - car_->dimensions_.wheelbase_) / 2 - CAR_MARGIN;
  p.y() = car_->dimensions_.width_ / 2 + CAR_MARGIN;
  visualization::DrawPoint(p, 0, local_viz_msg_); // Back-left corner
  p.x() = -(car_->dimensions_.length_ - car_->dimensions_.wheelbase_) / 2 - CAR_MARGIN;
  p.y() = -1 * (car_->dimensions_.width_ / 2 + CAR_MARGIN);
  visualization::DrawPoint(p, 0, local_viz_msg_); // Back-right corner
  p.x() = (car_->dimensions_.length_ + car_->dimensions_.wheelbase_) / 2 + CAR_MARGIN;
  p.y() = car_->dimensions_.width_ / 2 + CAR_MARGIN;
  visualization::DrawPoint(p, 0, local_viz_msg_); // Front-left corner
  p.x() = (car_->dimensions_.length_ + car_->dimensions_.wheelbase_) / 2 + CAR_MARGIN;
  p.y() = -1 * (car_->dimensions_.width_ / 2 + CAR_MARGIN);
  visualization::DrawPoint(p, 0, local_viz_msg_); // Front-right corner

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // The control iteration goes here.
  // The latest observed point cloud is accessible via "point_cloud_"

  // . regular TOC
  // controllers::time_optimal_1D::Command command {controller_->generateCommand(point_cloud_, robot_vel_(0))};
  // . with latency compensation
  controllers::time_optimal_1D::Command command {latency_controller_->generateCommand(point_cloud_, robot_vel_(0), last_msg_timestamp_)};

  // Eventually, you will have to set the control values to issue drive commands:
  // drive_msg_.curvature = 0.0;
  // drive_msg_.velocity = 0.1;
  drive_msg_.curvature = command.curvature;
  drive_msg_.velocity = command.velocity;

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();

  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);

  // std::cout << "End of loop" << std::endl;
}

}  // namespace navigation
