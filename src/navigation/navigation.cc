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
#include "navigation/path_options.h"

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

  robot_config_ = NavigationParams();
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
  last_msg_timestamp_ = time;                       
}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.

  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // Visualize pointcloud
  // for (int i = 0; i < (int)point_cloud_.size(); i++) {
  //   Vector2f p = point_cloud_[i];
  //   visualization::DrawCross(p, 0.01, 0, local_viz_msg_);
  // }

  // Visualize car
  visualization::DrawRectangle(Vector2f(car_->dimensions_.wheelbase_/2, 0),
      car_->dimensions_.length_, car_->dimensions_.width_, 0, 0x00FF00, local_viz_msg_);
  
  // Visualize margin
  visualization::DrawRectangle(Vector2f(car_->dimensions_.wheelbase_/2, 0),
      car_->dimensions_.length_ + 2 * CAR_MARGIN, car_->dimensions_.width_ + 2 * CAR_MARGIN, 0, 0xFA1600, local_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // The control iteration goes here.
  // The latest observed point cloud is accessible via "point_cloud_"

  // . regular TOC
  // controllers::time_optimal_1D::Command command {controller_->generateCommand(point_cloud_, robot_vel_(0))};
  // . with latency compensation
  std::vector<path_options::PathOption> path_options = path_options::samplePathOptions(31, point_cloud_, robot_config_);
  int best_path_index = path_options::selectPath(path_options);
  std::cout << "==========" << std::endl;
  std::cout << "\tCurv: " << path_options[best_path_index].curvature << "\tFpl: " << path_options[best_path_index].free_path_length << "\tClr: " << path_options[best_path_index].clearance << std::endl;

  path_options::PathOption best_path;
  std::vector<path_options::PathOption> path_candidates;
  controllers::time_optimal_1D::Command command {latency_controller_->generateCommand(point_cloud_, robot_vel_(0), last_msg_timestamp_, path_candidates, best_path, robot_config_)};

  std::cout << "\tCurv: " << best_path.curvature << "\tFpl: " << best_path.free_path_length << "\tClr: " << best_path.clearance << std::endl;

  // std::cout << "Returned " << path_candidates.size() << " paths!" << std::endl;
  // std::cout << "Command: " << command.curvature << std::endl;
  // std::cout << "Clearance of best path: " << best_path.clearance << std::endl;
  // std::vector<float> regimes {0.2, 0.35, 0.5, 1}; // curvature
  // std::vector<float> regimes {1.0, 2.0, 5.0, 8.0}; // free path length
  std::vector<float> regimes {0.03, 0.1, 0.25, 0.5}; // clearance
  // std::vector<float> regimes {0.5, 1.0, 2.0, 3.0}; // clearance
  // for (const auto &path : path_candidates) {
  for (const auto &path : path_options) {
    uint32_t color;
    if (std::abs(path.clearance) < regimes[0]) {
      color = 0xff0000;
    } else if (std::abs(path.clearance) < regimes[1]) {
      color = 0xff9933;
    } else if (std::abs(path.clearance) < regimes[2]) {
      color = 0xffff00;
    } else if (std::abs(path.clearance) < regimes[3]) {
      color = 0x66ff66;
    } else {
      color = 0x0000ff;
    }
    visualization::DrawPathOption(path.curvature,
                                  path.free_path_length,
                                  path.clearance,
                                  color, //32762,
                                  false,
                                  local_viz_msg_);
  }
  // visualization::DrawPathOption(best_path.curvature,
  //                                 best_path.free_path_length,
  //                                 best_path.clearance,
  //                                 10000,
  //                                 true,
  //                                 local_viz_msg_);
  visualization::DrawPathOption(path_options[best_path_index].curvature,
                                  path_options[best_path_index].free_path_length,
                                  path_options[best_path_index].clearance,
                                  10000,
                                  true,
                                  local_viz_msg_);

  // Eventually, you will have to set the control values to issue drive commands:
  drive_msg_.curvature = command.curvature;
  drive_msg_.velocity = command.velocity;

  // float current_speed = robot_vel_.norm();
  // drive_msg_.curvature = path_options[best_path_index].curvature;
  // drive_msg_.velocity = path_options::run1DTimeOptimalControl(path_options[best_path_index].free_path_length, current_speed, robot_config_);

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
