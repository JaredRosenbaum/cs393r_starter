//========================================================================
/*!
\file    global_planner.cc
\brief   Global planner implementation based on the RRT*. This employs Rapidly Exploring Random Trees.
\author  Daniel Meza, Jared Rosenbaum, Steven Swanbeck, (C) 2024
*/
//========================================================================

#include "global_planner.h"

namespace global_planner {

Global_Planner::Global_Planner(const vector_map::VectorMap map, ros::NodeHandle* n) :
  map_(map),
  goal_(0, 0),
  goal_threshold_(0.5),
  goal_reached_(false),
  graph_resolution_(0.3),
  collision_check_width_(0.05),
  sample_buffer_(5.0),
  optimization_radius_(1.5) {
    node_map_.clear();
    viz_pub_ = n->advertise<amrl_msgs::VisualizationMsg>("visualization", 1);
    viz_msg_ = visualization::NewVisualizationMessage(
      "map", "global_planner");
  }

void Global_Planner::ClearPath(void) {
  goal_reached_ = false;
  node_map_.clear();
  path_.clear();

  // Restart visualization
  visualization::ClearVisualizationMsg(viz_msg_);
  viz_pub_.publish(viz_msg_);
}

void Global_Planner::SetRobotLocation(const Eigen::Vector2f loc) {
  // Create starting node at the robot location
  Node node;
  node.id = 0;
  node.parent = 0;  // Top of tree, parent is oneself
  node.loc = loc;
  node.cost = 0;

  // Add to map of nodes
  node_map_.insert({node.id, node});

  std::cout << "[Global_Planner] Set robot location at (" << loc[0] << ", " << loc[1] << ")" << std::endl;
}

void Global_Planner::SetGoalLocation(const Eigen::Vector2f loc) {
  goal_ = loc;
  goal_reached_ = false;

  std::cout << "[Global_Planner] Set goal location at (" << loc[0] << ", " << loc[1] << ")" << std::endl;
}

bool Global_Planner::CalculatePath(unsigned int max_iterations) {
  std::cout << "[Global_Planner] Calculating path... " << std::endl;

  // Draw goal
  visualization::DrawCross(goal_, 0.1, 0xb30b0b, viz_msg_);

  // Loop performing RRT* path planning algorithm
  counter_ = 0;   // Exit condition if goal is unreachable
  while (counter_ < max_iterations && !goal_reached_) {
    counter_++;

    // Sample a random point in the state space
    Eigen::Vector2f sampled_loc = SamplePoint(node_map_.at(0).loc, goal_);

    // Find closest node to sampled point, if possible
    Node closest_node = FindClosestNode(sampled_loc);

    // Project closest node in the direction of the sampled point and create a new node
    Node new_node = CreateChildNode(closest_node, sampled_loc);

    // Check for any collisions with the map
    if (CheckMapCollision(new_node.loc, node_map_.at(new_node.parent).loc)) {
      continue;   // obstacle found, retry!
    }

    // If possible, optimize path
    OptimizePathToNode(&new_node);

    // Finally, add node to the node_map
    node_map_.insert({new_node.id, new_node});

    visualization::DrawPoint(new_node.loc, 0, viz_msg_);
    visualization::DrawLine(new_node.loc, node_map_.at(new_node.parent).loc, 0x1ac211, viz_msg_);

    viz_msg_.header.stamp = ros::Time::now();
    viz_pub_.publish(viz_msg_);

    // Exit when goal is reached
    float dist_to_goal = (new_node.loc - goal_).norm();
    if (dist_to_goal <= goal_threshold_) {
      std::cout << "[Global_Planner] Reached goal after " << counter_ << " iterations" << std::endl;
      goal_reached_ = true;

      // std::cout << "Press Enter to continue..." << std::endl;
      // std::cin.get();

      // Construct path to goal
      ConstructPath(new_node);
      return goal_reached_;
    }
  }

  std::cout << "[Global_Planner] Could not reach goal after " << counter_ << " iterations" << std::endl;

  return goal_reached_;
}

std::vector<Eigen::Vector2f> Global_Planner::GetPath(void) {
  return path_;
}

Eigen::Vector2f Global_Planner::SamplePoint(const Eigen::Vector2f robot_loc, const Eigen::Vector2f goal_loc) {
  // Calculate search space based on distance from robot location to goal
  Eigen::Vector2f midpoint = (robot_loc - goal_loc) / 2 + goal_loc;
  float search_radius = (robot_loc - goal_loc).norm() + sample_buffer_;

  // Randomly sample a point from the range in x and y
  Eigen::Vector2f point(
    rng_.UniformRandom(midpoint[0] - search_radius, midpoint[0] + search_radius),
    rng_.UniformRandom(midpoint[1] - search_radius, midpoint[1] + search_radius)
  );

  return point;
}

Node Global_Planner::FindClosestNode(const Eigen::Vector2f loc) {
  float min_dist = 1000;
  Node closest_node = node_map_.at(0);  // Default closest node is tree top

  // Loop through node map to find closest node
  for (auto const& index : node_map_) {
    Node node = index.second;   // retrieve node from node_map

    // Check for a smaller distance and update
    float dist = (node.loc - loc).norm();
    if (dist < min_dist) {
      min_dist = dist;
      closest_node = node;
    }
  }

  return closest_node;
}

bool Global_Planner::CheckMapCollision(const Eigen::Vector2f point1, const Eigen::Vector2f point2) {
  // Create line between node and its parent. Note we are checking two parallel lines that are very close to each other to handle the scenario where parts of the map are not connected (there's a tiny gap)
  float theta = atan2((point2[1] - point1[1]), (point2[2] - point1[2]));
  float dx = collision_check_width_ / 2 * cos(90 - theta);
  float dy = collision_check_width_ / 2 * sin(90 - theta);
  line2f line1(point1[0] - dx, point1[1] + dy,
               point2[0] - dx, point2[1] + dy);
  line2f line2(point1[0] + dx, point1[1] - dy,
               point2[0] + dx, point2[1] - dy);

  // Loop through map lines checking for instersections
  bool collision_flag = false;
  for (size_t j = 0; j < map_.lines.size(); ++j) {
    const line2f map_line = map_.lines[j];

    // Check for instersection
    Eigen::Vector2f intersection_point;
    // bool intersects = map_line.Intersection(line, &intersection_point);
    bool intersects1 = map_line.Intersection(line1, &intersection_point);
    bool intersects2 = map_line.Intersection(line2, &intersection_point);

    // Intersect found
    if (intersects1 || intersects2) {
      collision_flag = true;
      break;
    }
  }

  return collision_flag;
}

Node Global_Planner::CreateChildNode(Node parent, const Eigen::Vector2f loc) {
  // Calculate the projection in the direction from parent node to loc
  Eigen::Vector2f vect = (loc - parent.loc);
  Eigen::Vector2f proj(
    graph_resolution_ / vect.norm() * vect[0],
    graph_resolution_ / vect.norm() * vect[1]
  );

  // Create new child node
  Node node;
  node.id = counter_;
  node.parent = parent.id;
  node.loc = parent.loc + proj;
  node.cost = parent.cost + proj.norm();

  return node;
}

void Global_Planner::OptimizePathToNode(Node* node) {
  float min_cost = node->cost;        // cost of current path
  unsigned int best_neighbor = node->parent;  // current parent

  // Search for all neighbors within a set radius
  for (auto const& index : node_map_) {
    Node neighbor = index.second;   // retrieve node from node_map

    // Ignore oneself
    if (neighbor.id == node->id) {
      continue;
    }

    // Calculate distance to neighbor and see if it is within optimization radius
    float dist_to_neighbor = (node->loc - neighbor.loc).norm();
    if (dist_to_neighbor < optimization_radius_) {
      // Calculate new path cost and check if its the best option
      float cost = neighbor.cost + dist_to_neighbor;
      if (cost < min_cost) {

        // Ensure this does not creates a collision with the map
        if (!CheckMapCollision(node->loc, neighbor.loc)) {
          // Update best neighbot
          best_neighbor = neighbor.id;
          min_cost = cost;
        }
      }
    }
  }

  // Update path based on new best neighbor, if different from current
  if (best_neighbor != node->parent) {
    node->parent = best_neighbor;
    node->cost = min_cost;
  }
}

void Global_Planner::ConstructPath(const Node goal_node) {
  unsigned int steps = 0;

  // Draw goal
  visualization::DrawCross(goal_, 0.1, 0xb30b0b, viz_msg_);

  // Loop backwards up the tree, constructing the path.
  Node curr_node = goal_node;  // Current node is last node in the path
  while (curr_node.id > 0) {   // Note that the initial node will not get added as the robot is already at that location
    // Draw path segment
    visualization::DrawPoint(curr_node.loc, 0, viz_msg_);
    visualization::DrawLine(curr_node.loc, node_map_.at(curr_node.parent).loc, 0x0d13bf, viz_msg_);
    
    path_.insert(path_.begin(), curr_node.loc);
    curr_node = node_map_.at(curr_node.parent);
    steps++;

    viz_pub_.publish(viz_msg_);
  }

  std::cout << "[Global_Planner] Constructed global path of length " << goal_node.cost << " with " << steps << " steps" << std::endl;
}

}
