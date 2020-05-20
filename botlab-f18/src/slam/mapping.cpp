#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>
int bound(int value, int maximum, int minimum);

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
{
    initialFlag = 1;
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////
    if(initialFlag){
        posePrev = pose;
        initialFlag = 0;
        return;
    }
    MovingLaserScan tempScan(scan, posePrev, pose);
    posePrev = pose;

    for(auto ele = tempScan.begin(); ele != tempScan.end(); ele++){
        double endX = ele->origin.x + ele->range*cos(ele->theta);
        double endY = ele->origin.y + ele->range*sin(ele->theta);
        Point<double> endPoint = {endX, endY};
        Point<double> startPoint = {ele->origin.x, ele->origin.y};
        Point<int> endGrid = global_position_to_grid_cell(endPoint, map);
        Point<int> startGrid = global_position_to_grid_cell(startPoint, map);

        if(!map.isCellInGrid(endGrid.x , endGrid.y)){
            continue;
        }
        breshenham2(startGrid.x, startGrid.y, endGrid.x, endGrid.y, map);
    }
}

void Mapping::breshenham(const int startGridX, const int startGridY, const int endGridX, const int endGridY, OccupancyGrid& map){
    int dx = abs(endGridX - startGridX);
    int dy = abs(endGridY - startGridY);
    int sx = startGridX < endGridX ? 1:-1;
    int sy = startGridY < endGridY ? 1:-1;
    int err = dx - dy;
    int x = startGridX;
    int y = startGridY;
    while(x != endGridX && y != endGridY){
        //updateOdds(x, y, map, 0);
        //map(x,y) = bound(map(x,y) - kMissOdds_ , 127,-127);
        int e2 = 2 * err;
        if(e2 >= -dy){
            err -= dy;
            x += sx;
        }
        if(e2 <= dx){
            err += dx;
            y += sy;
        }
        map(x,y) = bound(map(x,y) - kMissOdds_ , 127,-127);
    }

    if(dx == 0 || dy == 0){
        map(endGridX,endGridY) = bound(map(endGridX,endGridY) + kHitOdds_, 127,-127);
    }
    // update endGrid odds
    updateOdds(endGridX, endGridY, map, 1);
    //map(endGridX,endGridY) = bound(map(endGridX,endGridY) + kHitOdds_ , 127,-127);
}

void Mapping::updateOdds(int x, int y, OccupancyGrid& map, int hitFlag){
    CellOdds oddsPrev = map.logOdds(x, y);
    CellOdds oddsNow;
    oddsNow = (hitFlag == 0) ?(oddsPrev - kMissOdds_):(oddsPrev + kHitOdds_ + kMissOdds_);
    //wrapOdds to (-127, 127)
    oddsNow = (oddsNow  < 127) ? oddsNow  : 127;
    oddsNow = (oddsNow  > -127) ? oddsNow  : -127;
    //printf("oddsNow: %d\n", oddsNow);
    map.setLogOdds(x, y, oddsNow);
}

void Mapping::breshenham2(int xStart, int yStart, int xEnd, int yEnd, OccupancyGrid& map){
    int Dx, Dy, dx, dy, x, y;
    //assert()
    x = xStart;
    y = yStart;
    Dx = xEnd - xStart;
    Dy = yEnd - yStart;
    dx = (Dx < 0) ? -1 : 1;
    dy = (Dy < 0) ? -1 : 1;
    Dx = abs(Dx);
    Dy = abs(Dy);
    if(Dx > Dy){
        int p = 2 * Dy - Dx;
        int const1, const2;
        const1 = 2 * Dy;
        const2 = 2* (Dy - Dx);
        while( x != xEnd){
            if( p < 0){
                p += const1;
            }
            else{
                p += const2;
                y += dy;
            }
            x += dx;
            map(x,y) = bound(map(x,y) - kMissOdds_, 127,-127);
        }
    }
    else{
        int p = 2 * Dx - Dy;
        int const1, const2;
        const1 = 2 * Dx;
        const2 = 2* (Dx - Dy);
        while( y != yEnd){
            if( p < 0){
                p += const1;
            }
            else{
                p += const2;
                x += dx;
            }
            y += dy;
            map(x,y) = bound(map(x,y) - kMissOdds_, 127,-127);
        }
    }
    if(Dx == 0 && Dy == 0){ 
        map(x,y) = bound(map(x,y) + kHitOdds_, 127,-127);
    }
    map(x,y) = bound(map(x,y) + 1*kHitOdds_ , 127,-127);
}

int bound(int value, int maximum, int minimum)
{
    return std::max(minimum, std::min(maximum, value));
}

