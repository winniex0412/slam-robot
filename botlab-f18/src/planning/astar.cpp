#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <common/grid_utils.hpp>
#include <common/point.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include <stdint.h>

#include <iterator> // for ostream_iterator
#include <iostream>
#include <typeinfo> // for debug printing types
#include <fstream>

double calcCost(const SearchParams& params, 
                double cellDistance, 
                const pose_xyt_t start, 
                const pose_xyt_t end, 
                const pose_xyt_t now, 
                double cellPerMeter){
  // cellDistance is in world coordinates
  // the poses are in cell coordinates

  double cost;
  // double world_cost;
  cost = sqrt(pow((end.x - now.x), 2) + pow((end.y - now.y),2));
  cost += sqrt(pow((start.x - now.x), 2) + pow((start.y - now.y),2));
  // if (cellDistance < params.maxDistanceWithCost){
  //   world_cost = cellPerMeter*pow(params.maxDistanceWithCost - cellDistance, params.distanceCostExponent);
  // }
  // std::cout<< "World cost: "<< world_cost << std::endl;
  // std::cout<< "Cell cost: "<< cost << std::endl;

  return cost;
};


void print_node(const Node input_node){ // debug function for printing node
  printf("Node \t x: %.2f \n", input_node.point.x);
  printf("\t y: %.2f \n", input_node.point.y);
  printf("\t   cost: %.2f \n", input_node.cost);
};


void printPath(const robot_path_t* path){
    std::cout << "\tFinal Path" << std::endl;
    std::cout << "\tSTART -->" << std::endl;
    for (unsigned int j=0; j< path->path.size(); j++){
      std::cout <<"\t " << path->path[j].x << "," << path->path[j].y <<std::endl;
    }
    std::cout << "\t --> GOAL"<< std::endl;

};

bool compareCost(const Node n1, const Node n2){
  return (n1.cost > n2.cost);
};

int compareNode(const Node n1, const Node n2){
  return n1.point.x == n2.point.x && n1.point.y == n2.point.y;
};

int world2cell(const ObstacleDistanceGrid& distances, float num, bool x_coord){
  if (x_coord){
    return static_cast<int>((num - distances.originInGlobalFrame().x) * distances.cellsPerMeter());
  }
  else{
    return static_cast<int>((num - distances.originInGlobalFrame().y) * distances.cellsPerMeter());
  }
};


float cell2world(const ObstacleDistanceGrid& distances, float num, bool x_coord){
  if (x_coord){
    return distances.originInGlobalFrame().x + num * distances.metersPerCell();;
  }
  else{
    return distances.originInGlobalFrame().y + num * distances.metersPerCell();;
  }
};

int writePathToFile(robot_path_t* path)
{
      std::ofstream myfile;
      myfile.open("a_star_final_path.csv");
      myfile << "x coord,y coord,theta" << std::endl;
      for (unsigned int j=0; j< path->path.size(); j++){
        myfile << path->path[j].x << "," << path->path[j].y << "," << path->path[j].theta <<std::endl;
      }
      myfile.close();

      std::cout << "\tFinished writing to file "<<std::endl;
      return 0;
}

int writeDGridToFile(const ObstacleDistanceGrid& distances)
{
      std::ofstream myfile;
      myfile.open("a_star_grid.csv");
      myfile << "Distance Grid " << std::endl;

      for (int x = 0; x < 1000; x++){ // distances.width_
          for (int y=0; y < 1000; y++){ // distances.height_
            if (distances.isCellInGrid(x, y)) {
              myfile << distances.operator()(x, y);
              myfile << ",";              
              // std::cout << distances.operator()(x, y);
              // std::cout << " | ";
            }
          }
          myfile<<"***"<<std::endl;
      }

      myfile.close();
      std::cout << "\tFinished writing grid to file "<<std::endl;
      return 0;
}

// After a path was found, return the correct ordering
robot_path_t get_final_path(const pose_xyt_t start,
                            const pose_xyt_t goal,
                            const pose_xyt_t start_world,
                            const pose_xyt_t goal_world,
                            const std::vector<Node> visited,
                            const ObstacleDistanceGrid& distances,
                            const SearchParams& params){
  robot_path_t path;
  path.utime = start.utime;
  Node current_node = visited.back();
  Node current_parent = visited.at(current_node.parent);

  // std::cout<< "\n***************\nFINAL PATH: "<<std::endl;

  while (current_node.parent!=0){

    current_node.point.x = cell2world(distances, current_node.point.x , true);
    current_node.point.y = cell2world(distances, current_node.point.y , false);

    current_node.point.x = (int)(current_node.point.x * 1000.0)/1000.0;
    current_node.point.y = (int)(current_node.point.y * 1000.0)/1000.0;
    current_node.point.theta = 0;

    path.path.emplace_back(current_node.point);
    current_node = visited.at(current_node.parent);
    current_parent = visited.at(current_node.parent);
  }

  current_node.point.x = cell2world(distances, current_node.point.x , true);
  current_node.point.y = cell2world(distances, current_node.point.y , false);
  current_node.point.x = (int)(current_node.point.x * 1000.0)/1000.0;
  current_node.point.y = (int)(current_node.point.y * 1000.0)/1000.0;
  current_node.point.theta = 0;

  path.path.emplace_back(current_node.point);

  std::reverse(path.path.begin(), path.path.end());
  path.path.back() = goal_world;
  path.path.front() = start_world;
  path.path_length = path.path.size();

  // average the path by num_avg
  // read average from 
  std::fstream myfile("../src/planning/averaging_num.txt", std::ios_base::in);
  int a;
  myfile >> a;

  unsigned int num_avg = a;
  robot_path_t averaged_path;
  averaged_path.path.push_back(path.path.front());

  for (unsigned int i=num_avg; i < path.path.size(); i+= num_avg){
    pose_xyt_t new_pose; 
    std::vector<double> vec_x;
    std::vector<double> vec_y;
    for (unsigned int j=0; j<num_avg; j++){
      vec_x.push_back(path.path[i-j].x);
      vec_y.push_back(path.path[i-j].y);
    }
    new_pose.x = std::accumulate(vec_x.begin(), vec_x.end(), 0.0) / vec_x.size();
    new_pose.y = std::accumulate(vec_y.begin(), vec_y.end(), 0.0) / vec_y.size();
    averaged_path.path.push_back(new_pose);
  }
  averaged_path.path.push_back(path.path.back());

  averaged_path.path_length = averaged_path.path.size();

  writePathToFile(&path);
  // writeDGridToFile(distances);
  printPath(&path);
  std::cout <<"\t AVERAGE VERSION ( "<< num_avg <<" ): " << std::endl;
  printPath(&averaged_path);
  std::cout << "Size of original path: "<< path.path.size() << " Size of averaged_path: " << averaged_path.path.size() << std::endl;
  return averaged_path;
};




robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{


    ////////////////// TODO: Implement your A* search here //////////////////////////
    // Input is start pose in world coordinates, goal pose in world coordinates
    // Inputs poses are converted to cells first
    // Then, everything is done in cell coordinates: cost, parents, etc.
    // In the very end, path poses should be converted back to world coordinates

    std::cout << "\tminDistanceToObstacle: " << params.minDistanceToObstacle << std::endl;
    std::cout << "\tmaxDistanceWithCost: " << params.maxDistanceWithCost << std::endl;
    std::cout << "\tdistanceCostExponent: " << params.distanceCostExponent << std::endl; 
     
    robot_path_t path;
    path.utime = start.utime;
    path.path.push_back(start);    
    path.path_length = path.path.size();

    // creating start and goal poses in cell coordinates instead of world coordinates
    pose_xyt_t start_cell_pose;
    start_cell_pose.x = world2cell(distances, start.x, true);
    start_cell_pose.y = world2cell(distances, start.y, false);

    pose_xyt_t goal_cell_pose;
    goal_cell_pose.x = world2cell(distances, goal.x, true);
    goal_cell_pose.y = world2cell(distances, goal.y, false);

    // printf("Searching for path from cell coords (%.2f, %.2f) to (%.2f, %.2f) \n", start_cell_pose.x, start_cell_pose.y, goal_cell_pose.x, goal_cell_pose.y);
    printf("\tSearching for path from global coords (%.2f, %.2f) to (%.2f, %.2f) \n", start.x, start.y, goal.x, goal.y);
    bool valid_start = distances.isCellInGrid(start_cell_pose.x, start_cell_pose.y);
    bool valid_goal = distances.isCellInGrid(goal_cell_pose.x, goal_cell_pose.y);

    if (!valid_start || !valid_goal){
      std::cout<< "\tINVALID START OR GOAL PROVIDED TO A* ALGORITHM " << std::endl;
      return path;
    }

    std::vector<Node> queue;
    std::vector<Node> visited;

    Node start_node;
    start_node.point = start_cell_pose;
    start_node.parent = 0; // no parent node
    double cellDistance = distances.operator()(start_node.point.x, start_node.point.x);
    start_node.cost = calcCost(params, cellDistance, start_cell_pose, goal_cell_pose, start_cell_pose, distances.cellsPerMeter()); 
    queue.emplace_back(start_node);
    visited.emplace_back(start_node);
    Node least_node;

    // Main Loop of Search Algorithm
    while (!queue.empty()){
      sort(queue.begin(), queue.end(), compareCost); // sort queue by cost. lowest cost is the last element
      least_node = queue.back();
      queue.pop_back(); // pop off least_cost_waypoint
      visited.emplace_back(least_node); // add least_cost_waypoint to waypoints_visited

      // END condition
      if (least_node.point.x == goal_cell_pose.x && least_node.point.y == goal_cell_pose.y){
          printf("\tPath found! \n");// path found, return path
          // std::cout<< "Final node parent is " << least_node.parent->point.x << " " << least_node.parent->point.y <<std::endl;
          robot_path_t final_path = get_final_path(start_cell_pose, goal_cell_pose, start, goal, visited, distances, params);
          return final_path;
        }

      for (float new_x =  least_node.point.x - 1; new_x < least_node.point.x + 2; new_x++){ // look at nodes in +- 1 x direction
        for (float new_y = least_node.point.y - 1; new_y < least_node.point.y + 2; new_y++){ // look at nodes in +- 1 y direction
          if (new_x != least_node.point.x || new_y != least_node.point.y){ // ignore least_node point
            if (distances.isCellInGrid(new_x, new_y) && 
                             distances.operator()(new_x, new_y) > params.minDistanceToObstacle){ // &&
                             // distances.operator()(new_x, new_y) < params.maxDistanceWithCost){ // check that new points are within Grid
              pose_xyt_t new_pose;
              new_pose.x = new_x;
              new_pose.y = new_y;
              new_pose.theta = least_node.point.theta;

              int parent_index = visited.size()-1;
              Node new_node;
              new_node.point = new_pose;
              new_node.parent = parent_index; // index of the new node parent
              cellDistance = distances.operator()(new_x, new_y);
              new_node.cost = calcCost(params, cellDistance, start_cell_pose, goal_cell_pose, new_pose, distances.cellsPerMeter());

              // check if new_node already visited or already in queue
              int contained = 0;
              for (unsigned int i=0; i < visited.size(); i++){
                if (compareNode(visited[i], new_node)){
                  // printf("Node contained in visited vector %.2f, %.2f \n", least_node.point.x+new_x, least_node.point.y+new_y);
                  contained = 1;
                }
              }
              for (unsigned int i=0; i < queue.size(); i++){
                if (compareNode(queue[i], new_node)){
                  // printf("Node contained in queue vector %.2f, %.2f \n", least_node.point.x+new_x, least_node.point.y+new_y);
                  if (new_node.cost < queue[i].cost){ // found a better path to this node
                    queue[i].cost = new_node.cost;
                  }
                  contained=1;
                }
              }
              
              if (!contained){ // if !contained, the new_node hasn't been checked yet
                queue.emplace_back(new_node);
              }

            }
            // else{
            //   printf("Cell not in grid (%.2f, %.2f \n)", least_node.point.x+new_x, least_node.point.y+new_y);
            // }
        }
      }
    }
  } // end of while loop

    printf("\tNO PATH FOUND!\n");
    // writePathToFile(&path);
    // writeDGridToFile(distances);
    return path;

}