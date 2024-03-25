//========================================================================
/*!
\file    global_planner.cc
\brief   TODO
\author  Daniel Meza, Jared Rosenbaum, Steven Swanbeck, (C) 2024
*/
//========================================================================

#include "global_planner.h"

namespace global_planner {

Global_Planner::Global_Planner(vector_map::VectorMap map) :
  map_(map),
  goal_(0, 0),
  goal_threshold_(0.25),
  goal_reached_(false),
  graph_resolution_(0.25),
  max_nodes_(1000000),
  optimization_radius_(1.0) {
    node_map_.clear();
  }

void Global_Planner::ClearPath() {
  node_map_.clear();
}

void Global_Planner::SetRobotPose(Eigen::Vector2f loc) {
  // Create starting node at the robot location
  Node node;
  node.id = 0;
  node.parent = 0;  // Parent is itself, starting node
  node.loc = loc;
  node.cost = 0;

  // Add to map of nodes
  node_map_.insert({node.id, node});
}

bool Global_Planner::CalculatePath(Eigen::Vector2f loc) {
  // Set new goal
  goal_ = loc;
  goal_reached_ = false;

  // Loop performing RRT* path planning algorithm
  counter_ = 0;   // Exit condition if goal is unreachable
  while (counter_ < max_nodes_) {
    // Sample a random point in the state space
    Eigen::Vector2f sampled_loc = SamplePoint();

    // Find closest node to sampled point
    Node closest_node = FindClosestNode(sampled_loc);

    // Project closest node in the direction of the sampled point and create a new node
    Node new_node = CreateChildNode(closest_node, sampled_loc);

    // If possible, optimize path and add node to the node map
    OptimizePathToNode(&new_node);
    node_map_.insert({new_node.id, new_node});

    // Exit when goal is reached
    float dist_to_goal = (new_node.loc - goal_).norm();
    if (dist_to_goal <= goal_threshold_) {
      goal_reached_ = true;
      break;
    }

    counter_++;
  }

  return goal_reached_;
}

Eigen::Vector2f Global_Planner::SamplePoint() {
  // TODO Select a random point in the map.
  // Ensure it is at least "graph resolution" away from any existing node. Otherwise select a different point.
  // This has the effect of selecting points in "unexplored" areas.

  Eigen::Vector2f point(0, 0);

  return point;
}

Node Global_Planner::FindClosestNode(Eigen::Vector2f loc) {
  float min_dist = 1000;
  Node closest_node = node_map_.at(0);

  // Loop through node map to find closest node that does not require passing through an obstacle to connect
  for (auto const& index : node_map_) {
    Node node = index.second;   // retrieve node from node_map

    // Check for a smaller distance
    float dist = (node.loc - loc).norm();

    if (dist < min_dist) {
      // Check that it does not pass through an obstacle
      line2f line(
        node.loc[0], node.loc[1],
        loc[0], loc[1]
      );

      // Loop through map lines checking for instersections
      for (size_t j = 0; j < map_.lines.size(); ++j) {
        const line2f map_line = map_.lines[j];


        // Check for instersection
        Eigen::Vector2f intersection_point;
        bool intersects = map_line.Intersection(line, &intersection_point);

        if (!intersects) {
          min_dist = dist;
          closest_node = node;
        }
      }
    }
  }

  return closest_node;
}

Node Global_Planner::CreateChildNode(Node parent, Eigen::Vector2f loc) {
  // Calculate the projection in the direction from parent node to loc
  Eigen::Vector2f vect = (loc - parent.loc);
  float x = parent.loc[0] + (graph_resolution_ / vect.norm() * vect[0]);
  float y = parent.loc[1] + (graph_resolution_ / vect.norm() * vect[1]);

  // Create new child node
  Node node;
  node.id = counter_;
  node.parent = parent.id;
  node.loc = parent.loc + Eigen::Vector2f(x, y);
  node.cost = parent.cost + vect.norm();

  return node;
}

void Global_Planner::OptimizePathToNode(Node* node) {
  float min_cost = node->cost;        // cost of current path
  unsigned int best_neighbor = node->parent;  // best neighbor for optimized path

  // Search for all neighbors within a set radius
  for (auto const& index : node_map_) {
    Node neighbor = index.second;   // retrieve node from node_map

    // Calculate distance to neighbor and see if its within optimization radius
    float dist_to_neighbor = (node->loc - neighbor.loc).norm();
    if (dist_to_neighbor < optimization_radius_) {
      // Calculate new path cost and check if its the best option
      float cost = neighbor.cost + dist_to_neighbor;
      if (cost < min_cost) {
        best_neighbor = neighbor.id;
        min_cost = cost;
      }
    }
  }

  // Update path based on new best neighbor, if different from current
  if (best_neighbor != node->parent) {
    node->parent = best_neighbor;
    node->cost = min_cost;
  }
}

}
