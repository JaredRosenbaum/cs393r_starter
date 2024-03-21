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
    Eigen::Vector2f goal_;     // Goal location (x,y)
    bool goal_reached_;        // Goal reached flag
    unsigned int max_nodes_;   // Maximum number of nodes to search

    std::map<unsigned int, Node> node_map_;   // Map of nodes
};

} // namespace global_planner
