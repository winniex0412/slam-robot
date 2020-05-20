#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp>
#include <iostream>


ObstacleDistanceGrid::ObstacleDistanceGrid(void)
: width_(0)
, height_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(20.0f)
{
}

void ObstacleDistanceGrid::printGrid()
{    // debug printing the entire ObstacleDistanceGrid
    std::cout<< "Printing Distance Grid ... " << std::endl;

    for (int x = 0; x < width_; x++){
        for (int y=0; y < height_; y++){    
    // for (int x = 0; x < 10; x++){
    //     for (int y=0; y < 10; y++){
            printf("%.2f | ", operator()(x, y));
        }
        std::cout<<" "<<std::endl;
    }
}

void ObstacleDistanceGrid::printOccupancyGrid(const OccupancyGrid& map)
{    // debug printing the entire ObstacleDistanceGrid
    std::cout<< "Printing Occupancy Grid ... " << std::endl;

    for (int y = 0; y < map.heightInCells(); y++){
        for (int x=0; x < map.widthInCells(); x++){
    // for (int y = 0; y < 10; y++){
    //     for (int x=0; x < 10; x++){
            if (isCellInGrid(x, y)){
                if (map.logOdds(x, y) >=0) printf("+");
                printf("%d | ", map.logOdds(x, y));
            }
        }
        std::cout<<" "<<std::endl;
    }
}

double ObstacleDistanceGrid::calcDist(int x1, int y1, int x2, int y2) const
{
    return pow(pow((float)(x2-x1), 2.0) + pow(float(y2-y1), 2.0), 0.5);
}

double ObstacleDistanceGrid::nearestObstacle(const OccupancyGrid& map, int x, int y) const
{
    // return 100000; // debug faster

    // This is a slow implementation that has to loop over the map
    // returns distance in cells from x,y to the nearest obstacle
    if (map.logOdds(x, y) >=0){ // the x,y coordinate itself is an obstacle
        return 0.0;
    }

    else{
        double shortest = pow(map.widthInCells(), 2)+pow(map.heightInCells(),2); // maximum distance across the grid
        for (int j=0; j < map.heightInCells(); j++){
            for (int i = 0; i < map.widthInCells(); i++){
                if (i!=x || j!=y){ // ignore the x,y cell itself
                    if (map.logOdds(i, j) >=0){
                        shortest = std::min(shortest, calcDist(x, y, i, j));
                        if (shortest<=1.0){ // short circuit since shortest will never be less than 1
                            return shortest;
                        }
                    }
                }
            }
        }
        return shortest;
    }
}


double ObstacleDistanceGrid::nearestObstacleBFS(const OccupancyGrid& map, int x, int y) const
{   
    // Uses a breadth-first search algorithm to find nearest obstacle distance quickly
    // returns distance in cells from cell x,y to the nearest obstacle
    if (map.logOdds(x, y) >=0){ // the x,y coordinate itself is an obstacle
        return 0.0;
    }

    std::tuple<int, int> start (x, y);
    std::deque<std::tuple<int, int>> queue;
    std::vector<std::tuple<int, int>> visited;
    queue.push_back(start);
    visited.push_back(start);
    bool found_obs = false;
    double shortest = pow(map.widthInCells(), 2)+pow(map.heightInCells(),2); // worst case distance
    int level = 0;
    // Main Loop of Search Algorithm
    while (!queue.empty()){
      std::tuple<int, int> least_node = queue.front(); // this should pop from front but doesn't work 
      queue.pop_front(); 
      visited.push_back(least_node); // add least_cost_waypoint to waypoints_visited
      // std::cout<<"least node " << std::get<0>(least_node) <<" "<< std::get<1>(least_node) << std::endl;
      // std::cout<<queue.size()<<std::endl;
      // END condition
      if (map.logOdds(std::get<0>(least_node), std::get<1>(least_node)) >= 0){
          found_obs = true;
          double dist = calcDist(std::get<0>(least_node), std::get<1>(least_node), std::get<0>(start), std::get<1>(start));
          shortest = std::min(shortest, dist);
          if (shortest <= 1.0) {return shortest;}
        }
      if (level > 1000){
        return calcDist(std::get<0>(least_node), std::get<1>(least_node), std::get<0>(start), std::get<1>(start));
        // return calcDist(level, level, 0, 0);
      }

      if (!found_obs){
        level ++; // expand another level of nodes
        for (double new_x = std::get<0>(least_node) - 1; new_x < std::get<0>(least_node) + 2; new_x++){ // look at nodes in +- 1 x direction
          for (double new_y = std::get<1>(least_node) - 1; new_y < std::get<1>(least_node) + 2; new_y++){ // look at nodes in +- 1 y direction
            if (new_x != std::get<0>(least_node) || new_y != std::get<1>(least_node)){ // ignore least_node point
              if (map.isCellInGrid(new_x, new_y)){

                  std::tuple<int,int> new_cell (new_x,new_y);
                  int contained = 0;

                  for (unsigned int i=0; i < visited.size(); i++){
                    if (std::get<0>(new_cell) == std::get<0>(visited[i]) && std::get<1>(new_cell) == std::get<1>(visited[i])) {
                        contained = 1;
                    }
                  }
                  for (unsigned int i=0; i < queue.size(); i++){
                    if (std::get<0>(new_cell) == std::get<0>(queue[i]) && std::get<1>(new_cell) == std::get<1>(queue[i])) {
                        contained = 1;
                    }
                  }
                // if !contained, the new_cell hasn't been checked yet
                if (!contained) {
                    queue.push_back(new_cell);
                }
              }
            }
          }
        }
      }
    }  // end of while loop
    if (!found_obs) {printf("NO OBSTACLES FOUND!\n");}
    return shortest;
}

double ObstacleDistanceGrid::nearestObstacleObs(const OccupancyGrid& map, int x, int y, std::vector<std::tuple<int, int>> obs) const
{   
    if (map.logOdds(x, y) >=0){ // the x,y coordinate itself is an obstacle
        return 0.0;
    }
    std::tuple<int, int> checkpt {x, y};
    double shortest = pow(map.widthInCells(), 2)+pow(map.heightInCells(),2);;
    for (unsigned int i=0; i < obs.size(); i++){
        shortest = std::min(shortest, calcDist(std::get<0>(checkpt), std::get<1>(checkpt), std::get<0>(obs[i]), std::get<1>(obs[i])));
    }
    return shortest;

}

void ObstacleDistanceGrid::setDistances(const OccupancyGrid& map)
{
    resetGrid(map);

    ///////////// TODO: Implement an algorithm to mark the distance to the nearest obstacle for every cell in the map.
    int num_obstacles = 0; // check if empty grid
    std::vector<std::tuple<int, int>> obs;
    bool useBFS = false;

    for (int y=0; y < map.heightInCells(); y++){
        for (int x = 0; x < map.widthInCells(); x++){
            if (map.logOdds(x, y) > 0.1){
                num_obstacles++;
            }
        }
    }

    if (num_obstacles > 0.8*map.widthInCells()* map.heightInCells()){
        std::cout<< "\t \t Using BFS because there are a lot of obstacles in this map  " << std::endl;
        useBFS = true;
    }
    // else {
        // if using OBS function, need to create list of obstacles
        for (int y=0; y < map.heightInCells(); y++){
            for (int x = 0; x < map.widthInCells(); x++){
                if (map.logOdds(x, y) >= 0){
                    obs.emplace_back(std::tuple<int, int> {x, y});
                }
            }
        }   
    // }

    for (int y=0; y < map.heightInCells(); y++){
        for (int x = 0; x < map.widthInCells(); x++){
            if (isCellInGrid(x, y)){
                // operator()(x, y) = metersPerCell_ * nearestObstacle(map, x, y); // this version is slow but passes test cases
                    if (useBFS){
                        operator()(x, y) = metersPerCell_ * nearestObstacleBFS(map, x, y); // this is fast in densely populated grids
                    }
                    else {// 
                        operator()(x, y) = metersPerCell_ * nearestObstacleObs(map, x, y, obs); // this version is much faster in sparse grids
                    }
            }
            else {
                std::cout<< "Cell not in grid: " << x << y << std::endl;
            }
        }
      } 
    // printGrid();
    // printOccupancyGrid(map);
    std::cout<<"Finished setting setDistances"<<std::endl;
}


bool ObstacleDistanceGrid::isCellInGrid(int x, int y) const
{
    return (x >= 0) && (x < width_) && (y >= 0) && (y < height_);
}


void ObstacleDistanceGrid::resetGrid(const OccupancyGrid& map)
{
    // Ensure the same cell sizes for both grid
    metersPerCell_ = map.metersPerCell();
    cellsPerMeter_ = map.cellsPerMeter();
    globalOrigin_ = map.originInGlobalFrame();
    
    // If the grid is already the correct size, nothing needs to be done
    if((width_ == map.widthInCells()) && (height_ == map.heightInCells()))
    {
        return;
    }
    
    // Otherwise, resize the vector that is storing the data
    width_ = map.widthInCells();
    height_ = map.heightInCells();
    
    cells_.resize(width_ * height_);
}
