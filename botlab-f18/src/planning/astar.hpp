#ifndef PLANNING_ASTAR_HPP
#define PLANNING_ASTAR_HPP

#include <lcmtypes/robot_path_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>


struct Node{
  // structure for nodes in the grid
  pose_xyt_t point;  // the pose_xyt_t itself
  int parent;        // array index of the parent node
  double cost;      // calculated cost for that node
};


class ObstacleDistanceGrid;

/**
* SearchParams defines the parameters to use when searching for a path. See associated comments for details
*/
struct SearchParams
{
    double minDistanceToObstacle;   ///< The minimum distance a robot can be from an obstacle before
                                    ///< a collision occurs
                                    
    double maxDistanceWithCost;     ///< The maximum distance from an obstacle that has an associated cost. The planned
                                    ///< path will attempt to stay at least this distance from obstacles unless it must
                                    ///< travel closer to actually find a path
                                    
    double distanceCostExponent;    ///< The exponent to apply to the distance cost, whose function is:
                                    ///<   pow(maxDistanceWithCost - cellDistance, distanceCostExponent)
                                    ///< for cellDistance > minDistanceToObstacle && cellDistance < maxDistanceWithCost
};

// Helper functions for A* star search
double calcCost(const SearchParams& params, double cellDistance, pose_xyt_t start, pose_xyt_t end, pose_xyt_t now, double cellPerMeter);
void print_node(Node input_node);
void printPath(const robot_path_t* path);
bool compareCost(Node n1, Node n2);
int compareNode(Node n1, Node n2);

int world2cell(const ObstacleDistanceGrid& distances, float num, bool x_coord);
float cell2world(const ObstacleDistanceGrid& distances, float num, bool x_coord);

int writePathToFile(robot_path_t* path);
int writeDGridToFile(const ObstacleDistanceGrid& distances);

robot_path_t get_final_path(const pose_xyt_t start,
                            const pose_xyt_t goal,
                            const pose_xyt_t start_world,
                            const pose_xyt_t goal_world,
                            std::vector<Node> visited,
                            const ObstacleDistanceGrid& distances,
                             const SearchParams& params);

/**
* search_for_path uses an A* search to find a path from the start to goal poses. The search assumes a circular robot
* 
* \param    start           Starting pose of the robot
* \param    goal            Desired goal pose of the robot
* \param    distances       Distance to the nearest obstacle for each cell in the grid
* \param    params          Parameters specifying the behavior of the A* search
* \return   The path found to the goal, if one exists. If the goal is unreachable, then a path with just the initial
*   pose is returned, per the robot_path_t specification.
*/
robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params);

#endif // PLANNING_ASTAR_HPP
