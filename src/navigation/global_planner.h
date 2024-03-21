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

#include "shared/math/geometry.h"

using geometry::line2f;

namespace global_planner {

struct Node {
  unsigned int id;
  unsigned int parent;
  Eigen::Vector2f loc;
  float cost;
  std::vector<Node> *neighbors;
};

class Global_Planner {
  public:
    Global_Planner();

    void SetRobotPose(Eigen::Vector2f loc, float angle);

    void CalculatePath(Eigen::Vector2f loc, float angle);

  private:
    Eigen::Vector2f SamplePoint();

    Node Global_Planner::FindClosestNode(Eigen::Vector2f loc);

    Node CreateChildNode(Node parent, Eigen::Vector2f loc);

    // Map of the environment.
    vector_map::VectorMap map_;

    unsigned int counter_;

    Eigen::Vector2f goal_;     // Goal location (x,y)
    float goal_threshold_;     // Set distance from goal for success
    bool goal_reached_;        // Goal reached flag

    float m_ = 3;

    float graph_resolution_;    // minimum distance between nodes
    unsigned int max_nodes_;   // Maximum number of nodes to search
    std::map<unsigned int, Node> node_map_;   // Map of nodes
};

} // namespace global_planner
