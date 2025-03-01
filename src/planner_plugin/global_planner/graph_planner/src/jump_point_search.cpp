/**
 * *********************************************************
 *
 * @file: jump_point_search.cpp
 * @brief: Contains the Jump Point Search(JPS) planner class
 * @author: Yang Haodong
 * @date: 2023-12-14
 * @version: 1.1
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include "jump_point_search.h"
#include <iostream>

namespace global_planner
{
/**
 * @brief Constructor
 * @param costmap the environment for path planning
 */
JumpPointSearch::JumpPointSearch(costmap_2d::Costmap2D* costmap) : GlobalPlanner(costmap)
{
}

/**
 * @brief Jump Point Search(JPS) implementation
 * @param start  start node
 * @param goal   goal node
 * @param expand containing the node been search during the process
 * @return true if path found, else false
 */
bool JumpPointSearch::plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand)
{
  start_ = start, goal_ = goal;

  // clear vector
  path.clear();
  expand.clear();

  // open list and closed list
  std::priority_queue<Node, std::vector<Node>, Node::compare_cost> open_list;
  std::unordered_map<int, Node> closed_list;

  open_list.push(start);

  // get all possible motions
  std::vector<Node> motions = Node::getMotion();

  // main loop
  while (!open_list.empty())
  {
    // pop current node from open list
    auto current = open_list.top();
    open_list.pop();

    // current node exist in closed list, continue
    if (closed_list.count(current.id()))
      continue;

    closed_list.insert(std::make_pair(current.id(), current));
    expand.push_back(current);

    // goal found
    if (current == goal)
    { 
      path = _convertClosedListToPath(closed_list, start, goal);
      return true;
    }

    // explore neighbor of current node
    for (const auto& motion : motions)
    {
      auto node_new = jump(current, motion);

      // node_new not exits or in closed list, continue
      if (node_new.id() == -1 || closed_list.count(node_new.id()))
        continue;
      node_new.set_pid(current.id());
      open_list.push(node_new);
    }
  }

  return false;
}

/**
 * @brief Calculate jump node recursively
 * @param point  current node
 * @param motion the motion that current node executes
 * @return jump node
 */
Node JumpPointSearch::jump(const Node& point, const Node& motion)
{
  Node new_point = point + motion;
  new_point.set_id(grid2Index(new_point.x(), new_point.y()));
  new_point.set_pid(point.id());
  new_point.set_h(helper::dist(new_point, goal_));

  // next node hit the boundary or obstacle
  if (new_point.id() < 0 || new_point.id() >= map_size_ ||
      costmap_->getCharMap()[new_point.id()] >= costmap_2d::LETHAL_OBSTACLE * factor_)
    return Node(-1, -1, -1, -1, -1, -1);

  // goal found
  if (new_point == goal_)
    return new_point;

  // diagonal
  if (motion.x() && motion.y())
  {
    // if exists jump point at horizontal or vertical
    Node x_dir = Node(motion.x(), 0, 1, 0, 0, 0);
    Node y_dir = Node(0, motion.y(), 1, 0, 0, 0);
    if (jump(new_point, x_dir).id() != -1 || jump(new_point, y_dir).id() != -1)
      return new_point;
  }

  // exists forced neighbor
  if (detectForceNeighbor(new_point, motion))
    return new_point;
  else
    return jump(new_point, motion);
}

/**
 * @brief Detect whether current node has forced neighbors
 * @param point  current node
 * @param motion the motion that current node executes
 * @return true if current node has forced neighbor else false
 */
bool JumpPointSearch::detectForceNeighbor(const Node& point, const Node& motion)
{
  int x = point.x();
  int y = point.y();
  int x_dir = motion.x();
  int y_dir = motion.y();
  auto costs = costmap_->getCharMap();

  // horizontal
  if (x_dir && !y_dir)
  {
    if (costs[grid2Index(x, y + 1)] >= costmap_2d::LETHAL_OBSTACLE * factor_ &&
        costs[grid2Index(x + x_dir, y + 1)] < costmap_2d::LETHAL_OBSTACLE * factor_)
      return true;
    if (costs[grid2Index(x, y - 1)] >= costmap_2d::LETHAL_OBSTACLE * factor_ &&
        costs[grid2Index(x + x_dir, y - 1)] < costmap_2d::LETHAL_OBSTACLE * factor_)
      return true;
  }

  // vertical
  if (!x_dir && y_dir)
  {
    if (costs[grid2Index(x + 1, y)] >= costmap_2d::LETHAL_OBSTACLE * factor_ &&
        costs[grid2Index(x + 1, y + y_dir)] < costmap_2d::LETHAL_OBSTACLE * factor_)
      return true;
    if (costs[grid2Index(x - 1, y)] >= costmap_2d::LETHAL_OBSTACLE * factor_ &&
        costs[grid2Index(x - 1, y + y_dir)] < costmap_2d::LETHAL_OBSTACLE * factor_)
      return true;
  }

  // diagonal
  if (x_dir && y_dir)
  {
    if (costs[grid2Index(x - x_dir, y)] >= costmap_2d::LETHAL_OBSTACLE * factor_ &&
        costs[grid2Index(x - x_dir, y + y_dir)] < costmap_2d::LETHAL_OBSTACLE * factor_)
      return true;
    if (costs[grid2Index(x, y - y_dir)] >= costmap_2d::LETHAL_OBSTACLE * factor_ &&
        costs[grid2Index(x + x_dir, y - y_dir)] < costmap_2d::LETHAL_OBSTACLE * factor_)
      return true;
  }

  return false;
}

}  // namespace global_planner