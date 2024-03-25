//========================================================================
/*!
\file    global_planner.h
\brief   TODO
\author  Daniel Meza, Jared Rosenbaum, Steven Swanbeck, (C) 2024
*/
//========================================================================

#pragma once

#include <vector>
#include <map>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"
#include "ros/package.h"
#include "vector_map/vector_map.h"
#include "shared/math/line2d.h"
#include "shared/util/random.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "visualization/visualization.h"

using geometry::line2f;

namespace global_planner {

struct Node {
  unsigned int id;
  Node* parent;
  Eigen::Vector2f loc;
  float cost;
};

class Global_Planner {
  public:
    Global_Planner(vector_map::VectorMap map, ros::NodeHandle* n);

    void ClearPath(void);

    void SetRobotLocation(Eigen::Vector2f loc);

    void SetGoalLocation(Eigen::Vector2f loc);

    bool CalculatePath(unsigned int max_iterations);

    std::vector<Eigen::Vector2f> GetPath(void);

  private:
    Eigen::Vector2f SamplePoint(Eigen::Vector2f robot_loc, Eigen::Vector2f goal_loc);

    Node FindClosestNode(Eigen::Vector2f loc);

    Node CreateChildNode(Node parent, Eigen::Vector2f loc);

    void OptimizePathToNode(Node* node);

    void ConstructPath(Node goal_node);

    ros::Publisher viz_pub_;
    amrl_msgs::VisualizationMsg viz_msg_;

    unsigned int counter_;

    vector_map::VectorMap map_;   // Map of the environment
    util_random::Random rng_;     // Random number generator

    Eigen::Vector2f goal_;     // Goal location (x,y)
    float goal_threshold_;     // Set distance from goal for success
    bool goal_reached_;        // Goal reached flag
    std::vector<Eigen::Vector2f> path_;   // Global path to goal

    float graph_resolution_;    // minimum distance between nodes
    std::map<unsigned int, Node> node_map_;   // Map of nodes

    float sample_buffer_;       // Search space buffer used for sampling random points
    float optimization_radius_; // Radius to search within for optimized path
};

} // namespace global_planner
