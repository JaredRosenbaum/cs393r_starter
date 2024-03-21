//========================================================================
/*!
\file    global_planner.cc
\brief   TODO
\author  Daniel Meza, Jared Rosenbaum, Steven Swanbeck, (C) 2024
*/
//========================================================================

#include "global_planner.h"

namespace global_planner {

  Global_Planner::Global_Planner() :
    goal_(0, 0),
    goal_reached_(false),
    max_nodes_(1000000) {
      node_map_.clear();
    }

  void Global_Planner::SetRobotPose(Eigen::Vector2f loc, float angle) {
    // Create starting node at the robot location
    Node node;
    node.id = 0;
    node.parent = 0;  // Parent is itself, starting node
    node.loc = loc;
    node.cost = 0;
    node.neighbors = new std::vector<Node>();

    // Add to map of nodes
    node_map_.insert({node.id, node});
  }

  void Global_Planner::CalculatePath(Eigen::Vector2f loc, float angle) {
    // Set new goal
    goal_ = loc;

    // Loop performing RRT* path planning algorithm
    unsigned int counter = 0;   // Exit condition if goal is unreachable
    Node current_node = node_map_.at(0);
    while (counter < max_nodes_) {
      // TODO


      counter++;
    }

  }

}
