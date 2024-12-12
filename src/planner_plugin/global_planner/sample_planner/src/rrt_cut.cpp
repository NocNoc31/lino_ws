/**
 * *********************************************************
 *
 * @file: rrt.cpp
 * @brief: Contains the Rapidly-Exploring Random Tree (RRT) planner class
 * @author: Yang Haodong
 * @date: 2022-10-27
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <cmath>
#include <random>
#include <iostream>

#include "rrt_cut.h"


namespace global_planner
{
/**
 * @brief  Constructor
 * @param   costmap   the environment for path planning
 * @param   sample_num  andom sample points
 * @param   max_dist    max distance between sample points
 */
RRTCut::RRTCut(costmap_2d::Costmap2D* costmap, int sample_num, double max_dist, int k)
  : RRTAStar(costmap, sample_num, max_dist, k)
{
}

/**
 * @brief RRT implementation
 * @param start         start node
 * @param goal          goal node
 * @param path          optimal path consists of Node
 * @param expand        containing the node been search during the process
 * @return  true if path found, else false
 */
bool RRTCut::plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand)
{
  path.clear();
  expand.clear();

  sample_list_.clear();
  // copy
  start_ = start, goal_ = goal;
  sample_list_.insert(std::make_pair(start.id(), start));
  expand.push_back(start);

  // extra 
  // int k = 5; // k sample point
  int n = 1;
  Node jump_init = start, jump_node; 
  std::vector<double> f_values;
  std::vector<Node> nearest_nodes, rand_nodes, esc_nodes;

  // jps
  jps_plan = std::make_shared<global_planner::JumpPointSearch>(costmap_);
  // std::cout << "Address of costmap_ in global planner: " << costmap_ << std::endl;
  // std::cout << "Address of costmap_ in JPS: " << jps_plan->getCostMap() << std::endl;

  // main loop
  int iteration = 0;
  while (iteration < sample_num_)
  { 
    f_values.clear();
    nearest_nodes.clear();
    rand_nodes.clear();

    // k loops
    for (int i = 0; i < k_; i++) {

      Node rand_node = _generateRandomNode();
      // obstacle
      if (costmap_->getCharMap()[rand_node.id()] >= costmap_2d::LETHAL_OBSTACLE * factor_)
        continue;
      // visited
      if (sample_list_.find(rand_node.id()) != sample_list_.end())
        continue;
      
      Node nearest_node = _getNearestNode(sample_list_, rand_node);

      // Calculate f_cost
      double g_cost = nearest_node.g();
      double h_cost = helper::dist(nearest_node, goal_); //helper::dist
      double f_cost = g_cost + h_cost;
      
      // Store for comparison
      f_values.push_back(f_cost);
      nearest_nodes.push_back(nearest_node);
      rand_nodes.push_back(rand_node);

    }

    Node best_nearest_node, best_rand_node;

    if (!f_values.empty()) {
        
      auto min_it = std::min_element(f_values.begin(), f_values.end());
      int min_idx = std::distance(f_values.begin(), min_it);

      best_nearest_node = nearest_nodes[min_idx];
      best_rand_node = rand_nodes[min_idx];
      
    }
    else {
    continue;
    }

    //check node 
    Node new_node = _getNewNode(best_nearest_node, best_rand_node);
    if (new_node.id() != -1){

      n = n + 1; 
      sample_list_.insert(std::make_pair(new_node.id(), new_node));
      expand.push_back(new_node);

      // std::cout << "Current value of n: " << n << std::endl;

      // goal found
          if (_checkGoal(new_node))
          { 
            std::cout << "Goal found "<< std::endl;

            //jps plan
            // std::vector<Node> path_segment;
            // std::vector<Node> expand_segment;
            // bool done2 = jps_plan->plan(start, goal, path_segment, expand_segment);

            // if(done2){
            
            //   if(path.empty()){
            //   path = path_segment;
            //   } else {
            //     // path.insert(path.end(), path_segment.begin() + 1, path_segment.end());
            //     n = n - 1;
            //     continue;
            //   } 

            //   // expand.insert(expand.end(), expand_segment.begin(), expand_segment.end()); 
            //   // jump_init = jump_node;

            // } else {
            //   n = n - 1;
            //   continue;
            // }

            //rrt_astar 
            path = _convertClosedListToPath(sample_list_, start, goal);
            return true;
          }

      // limited node 
      if( n % 50 == 0){
        double min_f_value = std::numeric_limits<double>::infinity();

        // check sample_list_
        for (const auto& entry : sample_list_) {
          Node node = entry.second;
          
          if(node.id() != start.id()){
            double g_cost = node.g();
            double h_cost = heuristic(node, goal_); //helper::dist
            double f_value = 0.5*g_cost + h_cost;

            if (f_value < min_f_value) {
              min_f_value = f_value;
              jump_node = node;
            }
          // std::cout << "Current value of min f: " << min_f_value << std::endl;
          }
        }
        // rounded
        // jump_node.set_x(std::round(jump_node.x()));
        // jump_node.set_y(std::round(jump_node.y()));

        // std::cout << "Jump node is obstacle?: " << ((costmap_->getCharMap()[jump_node.id()] >= costmap_2d::LETHAL_OBSTACLE * factor_) ? "Yes" : "No") << std::endl;
        if (costmap_->getCharMap()[jump_node.id()] >= costmap_2d::LETHAL_OBSTACLE * factor_){
          n = n - 1;
          continue;
        }
        else{

          // jps
          // std::vector<Node> path_segment;
          // std::vector<Node> expand_segment;

          std::cout << "Start Node: (" << start.x() << ", " << start.y() << ")" << std::endl;
          std::cout << "Jump Node: (" << jump_node.x() << ", " << jump_node.y() << ")" << std::endl;
          std::cout << "Goal Node: (" << goal.x() << ", " << goal.y() << ")" << std::endl;

          jump_node.set_id(jps_plan->grid2Index(jump_node.x(), jump_node.y()));

          //jps_plan
          // bool done1 = jps_plan->plan(start, jump_node, path_segment, expand_segment);
          // // std::cout << "JPS 1 planning completed. Result: " << (done1 ? "Success" : "Failure") << std::endl;
          // if(done1){
          //   // std::cout << "Has Path ?: " << (done1 ? "Yes" : "No") << std::endl;
          //   if(path.empty()){
          //     path = path_segment;
          //   } else {
          //     // path.insert(path.end(), path_segment.begin() + 1, path_segment.end());
          //     n = n - 1;
          //     continue;
          //   }
          //   // jump_init = jump_node; 
          //   // sub_goal = jump_node;  
          //   // std::cout << "Has Path ? ?: " << ((!path.empty()) ? "Yes" : "No") << std::endl;
          //   return true;
          // } else {
          //   n = n - 1;
          //   continue;
          // }

          //rrt_astar
          path = _convertClosedListToPath(sample_list_, start, jump_node);
          return true;
        }
      }
    }  
    else{ 
      // std::cout << "Obstacle collision ! " << std::endl;
      continue;
    }

    iteration++;
  }
  return false;
}

double RRTCut::heuristic(const Node& n1, const Node& n2){
  return std::abs(n1.x() - n2.x()) + std::abs(n1.y() - n2.y());
}

// bool RRTCut::escape(const Node& start, const Node& goal, Node& sub_goal){

// }

// Node RRTCut::_getForceNeighbor(const Node& cur_node, const Node& goal, const Node& motion){

// }

// Node RRTCut::_detectForceNeighbor(const Node& cur_node, const Node& motion){
//   int x = cur_node.x();
//   int y = cur_node.y();
//   int x_dir = motion.x();
//   int y_dir = motion.y();
//   auto costs = costmap_->getCharMap();

//   Node neighbor;
//   // horizontal
//   if (x_dir && !y_dir)
//   {
//     if (costs[grid2Index(x, y + 4)] >= costmap_2d::LETHAL_OBSTACLE * factor_ &&
//         costs[grid2Index(x + x_dir, y + 4)] < costmap_2d::LETHAL_OBSTACLE * factor_){
//           neighbor.set_x(x + x_dir);
//           neighbor.set_y(y + 4);
//           return neighbor; }
//     if (costs[grid2Index(x, y - 4)] >= costmap_2d::LETHAL_OBSn TACLE * factor_ &&
//         costs[grid2Index(x + x_dir, y - 4)] < costmap_2d::LETHAL_OBSTACLE * factor_){
//           neighbor.set_x(x + x_dir);
//           neighbor.set_x(y - 4);
//           return neighbor; }
//   }

//   // vertical
//   if (!x_dir && y_dir)
//   {
//     if (costs[grid2Index(x + 4, y)] >= costmap_2d::LETHAL_OBSTACLE * factor_ &&
//         costs[grid2Index(x + 4, y + y_dir)] < costmap_2d::LETHAL_OBSTACLE * factor_){
//           neighbor.set_x(x + 4);          
//           neighbor.set_y(y + y_dir);          
//           return neighbor; }
//     if (costs[grid2Index(x - 4, y)] >= costmap_2d::LETHAL_OBSTACLE * factor_ &&
//         costs[grid2Index(x - 4, y + y_dir)] < costmap_2d::LETHAL_OBSTACLE * factor_){
//           neighbor.set_x(x - 4);          
//           neighbor.set_y(y + y_dir);
//           return neighbor; }
//   }

//   // diagonal
//   if (x_dir && y_dir)
//   {
//     if (costs[grid2Index(x - 3*x_dir, y)] >= costmap_2d::LETHAL_OBSTACLE * factor_ &&
//         costs[grid2Index(x - 3*x_dir, y + 3*y_dir)] < costmap_2d::LETHAL_OBSTACLE * factor_){
//           neighbor.set_x(x - 3*x_dir);          
//           neighbor.set_y(y + 3*y_dir);
//           return neighbor; }
//     if (costs[grid2Index(x, y - 3*y_dir)] >= costmap_2d::LETHAL_OBSTACLE * factor_ &&
//         costs[grid2Index(x + 3*x_dir, y - 3*y_dir)] < costmap_2d::LETHAL_OBSTACLE * factor_){
//           neighbor.set_x(x + 3*x_dir);          
//           neighbor.set_y(y - 3*y_dir);
//           return neighbor; }
//   }

//   return neighbor;
// }

// Node RRTCut::_getMotion(const Node start, const Node goal){
  
// }
// }

//----------------------------------------------------------------------------------------------------------
bool RRTCut::escape(const Node& start, const Node& goal, Node& sub_goal){
  // path.clear();
  // expand.clear();

  std::cout << "Start escape function "<< std::endl;
  // escape_list_.clear();
  // copy
  start_ = start, goal_ = goal;
  // escape_list_.insert(std::make_pair(start.id(), start));
  // expand.push_back(start);

  //jps plan
  jps_plan = std::make_shared<global_planner::JumpPointSearch>(costmap_);
  double cmin = 0.5, check_dist = -1000; //0.005
  sub_goal = goal;
  bool near_goal = false;
  // std::cout << "Start get confidence"<< std::endl;
  
  double confidence = _getConfidence(goal_);
  // std::cout << "Confidence = "<< confidence << std::endl;

  for(double theta1 = 0; theta1 < 360; theta1 = theta1 + 60){
    Node q1 = _getState(goal_, confidence, theta1);
    double d1 = _getConfidence(q1);
    // std::cout << "d1 = "<< d1 << std::endl;

    if(d1 <= cmin) continue;
    if(_isAnyObstacleInPath2(goal_, q1)) {
      // std::cout << "obstacle d1 " << std::endl;
      continue;}
    
    for(double theta2 = 0; theta2 < 360; theta2 = theta2 + 60){
      Node q2 = _getState(q1, d1, theta2);
      double d2 = _getConfidence(q2);
      // std::cout << "d2 = "<< d2 << std::endl;

      if((d2 <= cmin) || (helper::dist(q2, goal_) <= confidence)) continue;
      if(_isAnyObstacleInPath2(q1, q2)) continue;

      for(double theta3 = 0; theta3 < 360; theta3 = theta3 + 60){
        Node q3 = _getState(q2, d2, theta3);
        double d3 = 1.2*_getConfidence(q3);
        // std::cout << "d3 = "<< d3 << std::endl;

        if((d3 <= cmin) || (helper::dist(q3, q1) <= d1)) continue;
        if(_isAnyObstacleInPath2(q2, q3)) continue;

        for(double theta4 = 0; theta4 < 360; theta4 = theta4 + 10){
          // double theta4 = 180;
          Node q4 = _getState(q3, d3, theta4);
          double d4 = _getConfidence(q4);
          // std::cout << "d4 = "<< d4 << std::endl;

          if((d4 <= cmin) || (helper::dist(q4, q2) <= d2)) continue;

          if(_isAnyObstacleInPath2(q3, q4)) continue;

          // std::cout << "near_goal: "<< near_goal << std::endl;

          if((!_isAnyObstacleInPath2(goal_, q4)) && (near_goal == false)){ //&& (helper::dist(start_, q4) >= d3) @
            continue; 
          } //&& helper::dist(start_, goal_) >= d3

          if(near_goal == true && _isAnyObstacleInPath2(goal_, q4)){
            continue; 
          }

          double check_dist_new = 2.5*helper::dist(q4, goal_) - helper::dist(q4, start_); //2.5

          if(check_dist_new >= check_dist){
            sub_goal.set_x(q4.x());
            sub_goal.set_y(q4.y());
            check_dist = check_dist_new;
          }
            
        }
        if((helper::dist(start_, sub_goal) <= 25)){
          // std::cout << "closed sub_goal"<< std::endl;
          near_goal = true;
        }
      }
    }
    
  }


  // check sub_goal
        
        // else{
        //   near_goal = false;
        // }
  if ( (!_isAnyObstacleInPath2(start_, goal_)) || (helper::dist(sub_goal, start_) <= 17)) //(helper::dist(sub_goal, start_) <= 12) || @
  {
    sub_goal = goal_;
    return true;
  }
  

  if(sub_goal.x() != goal_.x()){
    sub_goal.set_id(jps_plan->grid2Index(sub_goal.x(), sub_goal.y()));
    std::cout << "Sub_goal found "<< std::endl;
    // std::cout << "Sub_goal: (" << sub_goal.x() << ", " << sub_goal.y() << ")" << std::endl;
    // std::cout << "Start_: (" << start.x() << ", " << start.y() << ")" << std::endl;
    if((!_isAnyObstacleInPath2(start_, goal_)) ){
            std::cout << "goal found"<< std::endl;
            sub_goal = goal_;
          }
    // std::vector<Node> path_sub;
    // std::vector<Node> expand_sub;
    // bool done = jps_plan->plan(start_, sub_goal, path_sub, expand_sub);
    return true;
  }
  else{
    std::cout << " No sub_goal return "<< std::endl;
    return false;
  }
}

double RRTCut::_getConfidence(const Node& goal){

  double cmax = 20, step_size = 1; // cmax = 2.5, step_size = 0.0045
  double confidence = std::numeric_limits<double>::infinity();

  struct Direction {
        double x_offset;
        double y_offset;
    };

  Direction directions[] = {
        { -step_size, 0.0 },   // Trái
        { step_size, 0.0 },    // Phải
        { 0.0, step_size },    // Lên
        { 0.0, -step_size }    // Xuống
    };

  for (const auto& dir : directions) 
    {
        Node current_point = goal, next_point;

        while (true) 
        {
            next_point.set_x(current_point.x() + dir.x_offset);
            next_point.set_y(current_point.y() + dir.y_offset);
            // std::cout << "Next point: (" << next_point.x() << ", " << next_point.y() << ")" << std::endl;
            //obstacle
            if (_isAnyObstacleInPath(current_point, next_point)) 
            {   
                // std::cout << "Colision" << std::endl;
                double distance = helper::dist(goal, current_point);
                // std::cout << "Distance: : " << distance << std::endl;

                // ROS_INFO("distance = %f", distance);
                if (distance < confidence) 
                {
                    confidence = distance;
                }
                break;
            }else{
              current_point.set_x(next_point.x());
              current_point.set_y(next_point.y());
              // std::cout << "Cur point: (" << current_point.x() << ", " << current_point.y() << ")" << std::endl;
            }
        }
    }

    if (confidence == std::numeric_limits<double>::infinity()) 
    {
        return -1.0; 
    }

    if (confidence >= cmax) confidence = cmax;

    return confidence;  
}

Node RRTCut::_getState(const Node& q, double confidence, double theta){

  Node q_close;
  q_close.set_x(q.x() + confidence*cos(theta*(M_PI/180)));
  // std::cout << "q.x() = "<< q.x() + confidence*cos(theta*(M_PI/180)) << std::endl;

  q_close.set_y(q.y() + confidence*sin(theta*(M_PI/180)));

  return q_close;
}

}  // namespace global_planner

