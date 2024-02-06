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
    // car_ (vehicles::UT_Automata()),
    // controller_ (controllers::time_optimal_1D::Controller(car_, TIME_STEP, CAR_MARGIN, MAX_CLEARANCE, CURVATURE_SAMPLING_INTERVAL)),
    // latency_controller_ (controllers::latency_compensation::Controller(car_, TIME_STEP, CAR_MARGIN, MAX_CLEARANCE, CURVATURE_SAMPLING_INTERVAL, TIME_STEP)),
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

  latency_controller_ = new controllers::latency_compensation::Controller(car_, TIME_STEP, CAR_MARGIN, MAX_CLEARANCE, CURVATURE_SAMPLING_INTERVAL, TIME_STEP);
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

  // // Visualize pointcloud
  // for (int i = 0; i < (int)point_cloud_.size(); i++) {
  //   Vector2f p = point_cloud_[i];
  //   visualization::DrawCross(p, 0.01, 0, local_viz_msg_);
  // }

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // The control iteration goes here.
  // The latest observed point cloud is accessible via "point_cloud_"

  // TODO Used for testing, remove later
  // controllers::time_optimal_1D::Command command{0.0, -0.35};
  // // std::cout << point_cloud_.size() << " points" << std::endl;
  // float fpl = controller_->calculateFreePathLength(point_cloud_, command.curvature);
  // command.velocity = controller_->calculateControlSpeed(robot_vel_(0), fpl);
  // std::cout << fpl << std::endl;
  // std::cout << "  " << command.velocity << std::endl;

  // Run the time optimal controller to calculate drive commands
  // controllers::time_optimal_1D::Command command {controller_->generateCommand(point_cloud_, robot_vel_(0))};



  // // & testing latency
  // auto cloud {utils::testing::generateTestCloud()};
  // // controllers::time_optimal_1D::Command command{0.0, -1.0};
  // float curvature {0.f};
  // float fpl = controller_->calculateFreePathLength(cloud, curvature);
  // std::cout << "Regular controller free path length: " << fpl << std::endl;

  // ros::Rate rate(100);
  // std::cout << "-----------------" << std::endl;
  // std::cout << point_cloud_.size() << " points are available from timestamp " << last_msg_timestamp_ << std::endl;

  // controllers::time_optimal_1D::Command command_past (0.f, 0.f);
  // command_past.velocity = 1.0;
  // latency_controller_->recordCommand(command_past);
  // rate.sleep();
  // latency_controller_->recordCommand(command_past);
  // rate.sleep();
  // latency_controller_->recordCommand(command_past);
  // rate.sleep();
  // latency_controller_->recordCommand(command_past);
  // rate.sleep();
  // command_past.curvature = 1.0;
  // latency_controller_->recordCommand(command_past);
  // rate.sleep();

  // // latency_controller_->projectState(last_msg_timestamp_);
  // float latency_fpl = latency_controller_->calculateFreePathLength(cloud, curvature, last_msg_timestamp_);

  // std::cout << "FPL:\n\tWithout Latency:\t" << fpl << "\n\tWith Latency:\t\t" << latency_fpl << std::endl;

  // // latency_controller_->generateCommand(cloud, curvature, last_msg_timestamp_);

  // std::cout << "Collection span: " << (latency_controller_->command_history_.back().timestamp - latency_controller_->command_history_.front().timestamp) << std::endl;
  // latency_controller_->command_history_.clear();
  // // & testing latency


  std::cout << "About to update controller..." << std::endl;
  std::cout << point_cloud_.size() << ", " << robot_vel_(0) << ", " << last_msg_timestamp_ << std::endl;
  // controllers::time_optimal_1D::Command command {controller_->generateCommand(point_cloud_, robot_vel_(0))};
  controllers::time_optimal_1D::Command command{latency_controller_->generateCommand(point_cloud_, robot_vel_(0), last_msg_timestamp_)};
  // command = latency_controller_->generateCommand(point_cloud_, robot_vel_(0), last_msg_timestamp_);

  std::cout << command.velocity << ", " << command.curvature << std::endl;

  // Eventually, you will have to set the control values to issue drive commands:
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

  std::cout << "End of loop" << std::endl;
}

}  // namespace navigation
