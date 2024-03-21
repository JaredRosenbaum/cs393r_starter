//========================================================================
/*!
\file    global_planner.cc
\brief   TODO
\author  Daniel Meza, Jared Rosenbaum, Steven Swanbeck, (C) 2024
*/
//========================================================================

#include "global_planner.h"`

namespace global_planner {

Global_Planner::Global_Planner() :
  goal_(0, 0),
  goal_threshold_(0.1),
  goal_reached_(false),
  max_nodes_(1000000),
  graph_resolution_(0.25) {
    // TODO map_ is already loaded by navigation, adjust so we simply use the same one!
    // map_.Load(map_file);
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
  counter_ = 0;   // Exit condition if goal is unreachable
  while (counter_ < max_nodes_) {
    // Sample a random point in the state space
    Eigen::Vector2f sampled_loc = SamplePoint();

    // Find closest node to sampled point
    Node closest_node = FindClosestNode(sampled_loc);

    // Project closest node in the direction of the sampled point and create a new node
    Node new_node = CreateChildNode(closest_node, sampled_loc);

    // If possible, optimize path
    // TODO

    counter_++;
  }

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
  for (auto const& [id, node] : node_map_) {
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
        Eigen::Vector2f instersection_point;
        bool intersects = map_line.Intersection(ray, &intersection_point);

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
  // TODO
  // Calculate direction of projection from parent to point
  Eigen::Vector2f vect = (loc - parent.loc);
  float x = parent.loc[0] + (graph_resolution_ / vect.norm() * vect[0]);
  float y = parent.loc[1] + (graph_resolution_ / vect.norm() * vect[1]);

  // Create new node
  Node node;
  node.id = 0;
  node.parent = parent.id;  // Parent is itself, starting node
  node.loc = parent.loc + Eigen::Vector2f(x, y);
  node.cost = 0;
  node.neighbors = new std::vector<Node>();
}

}
