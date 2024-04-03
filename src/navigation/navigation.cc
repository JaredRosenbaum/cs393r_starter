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
#include "navigation/path_generation.h"

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

  //* instantiating the local planner
  // Simple path for testing purposes
  // for (int i = 0; i < 20; i++){
  //   Vector2f point(i, rand()%3);
  //   if (rand()%3 == 1){
  //     point.y() *= -1;
  //   }
  //   testing_path.push_back(point);
  // }
  // carrot_planner_ = new local_planners::CarrotPlanner(STICK_LENGTH, GOAL_TOL, PATH_DEV_TOL);
  smoothed_planner_ = new local_planners::SmoothedPlanner(map_, STICK_LENGTH, GOAL_TOL, PATH_DEV_TOL);

  // robot_config_ = NavigationParams();
  // +

  // instantiate a global planner
  global_planner_ = new global_planner::Global_Planner(map_, n, GOAL_THRESHOLD, GRAPH_RESOLUTION, COLLISION_PROXIMITY, SAMPLE_BUFFER, OPTIMIZATION_RADIUS);
  global_path_found_ = false;
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
  global_path_found_ = global_planner_->CalculatePath(MAX_SAMPLING_ITERATIONS);

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

  // std::cin.get();

  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // Visualize car
  visualization::DrawRectangle(Vector2f(car_->dimensions_.wheelbase_/2, 0),
      car_->dimensions_.length_, car_->dimensions_.width_, 0, 0x00FF00, local_viz_msg_);
  
  // Visualize margin
  visualization::DrawRectangle(Vector2f(car_->dimensions_.wheelbase_/2, 0),
      car_->dimensions_.length_ + 2 * CAR_MARGIN, car_->dimensions_.width_ + 2 * CAR_MARGIN, 0, 0xFA1600, local_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;
  if (!global_path_found_) return;

  // . regular TOC
  // controllers::time_optimal_1D::Command command {controller_->generateCommand(point_cloud_, robot_vel_(0))};
  testing_path = global_planner_->GetPath();
  // carrot_planner_->populatePath(testing_path);
  smoothed_planner_->populatePath(testing_path);

  //Temp visualization of path
  for (std::size_t i=0; i<=testing_path.size()-2; i++){
        visualization::DrawLine(testing_path[i],testing_path[i+1],0x38114a,global_viz_msg_);
        visualization::DrawCross(testing_path[i],0.025,0x38114a,global_viz_msg_);
  }

  // Vector2f goal {carrot_planner_->feedCarrot(robot_loc_)};
  // if (carrot_planner_->reachedGoal(robot_loc_, nav_goal_loc_)) {global_path_found_ = false;}
  // if (!carrot_planner_->planStillValid(robot_loc_)) {global_path_found_ = false;}
  geometry::line2f l1;
  geometry::line2f l2;
  Vector2f goal {smoothed_planner_->interpolatePath(robot_loc_, 0.1, l1, l2)};
  if (smoothed_planner_->reachedGoal(robot_loc_, nav_goal_loc_)) {global_path_found_ = false;}
  if (!smoothed_planner_->planStillValid(robot_loc_)) {
    global_path_found_ = false;
    global_planner_->ClearPath();
    global_planner_->SetRobotLocation(robot_loc_);
    global_planner_->SetGoalLocation(nav_goal_loc_);
    global_path_found_ = global_planner_->CalculatePath(500000);
  }

  visualization::DrawLine(l1.p0, l1.p1, 0xF633FF, global_viz_msg_);
  visualization::DrawLine(l2.p0, l2.p1, 0xF633FF, global_viz_msg_);

  visualization::DrawCross(goal,0.25,0x38114a,global_viz_msg_);
  // .Transform goal to robot frame
  goal -= robot_loc_;
  Vector2f temp(goal.x(), goal.y());
  goal.x() = temp.x()*cos(-robot_angle_) - temp.y()*sin(-robot_angle_);
  goal.y() = temp.y()*cos(-robot_angle_) + temp.x()*sin(-robot_angle_);
  visualization::DrawCross(goal,0.25,0x38114a,local_viz_msg_);
  
  // . with latency compensation
  path_generation::Path best_path;
  std::vector<path_generation::Path> path_options;
  Eigen::Vector2f global_goal {testing_path[testing_path.size() - 1] - robot_loc_};
  controllers::time_optimal_1D::Command command {latency_controller_->generateCommand(point_cloud_, robot_vel_(0), last_msg_timestamp_, path_options, best_path, goal, global_goal)};

  // std::cout << "==========" << std::endl;
  // std::cout << "\tCurv: " << best_path.curvature << "\tFpl: " << best_path.free_path_length << "\tClr: " << best_path.clearance << std::endl;

  // . Draw possible paths
  std::vector<float> regimes {0.03, 0.1, 0.25, 0.5}; // clearance
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
  // . Draw best path
  visualization::DrawPathOption(best_path.curvature,
                                  best_path.free_path_length,
                                  best_path.clearance,
                                  10000,
                                  true,
                                  local_viz_msg_);

  // Eventually, you will have to set the control values to issue drive commands:
  drive_msg_.curvature = command.curvature;
  drive_msg_.velocity = command.velocity;
  // drive_msg_.velocity = 0;

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
