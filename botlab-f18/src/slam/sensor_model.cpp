#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/grid_utils.hpp>

bool breshenham2(int xStart, int yStart, int xEnd, int yEnd, const OccupancyGrid& map);

SensorModel::SensorModel(void)
{
    ///////// TODO: Handle any initialization needed for your sensor model
}


double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{
    ///////////// TODO: Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
    //sample.pose.x 
    //return 1;
    int ii = 0;
    MovingLaserScan tempScan(scan, sample.parent_pose, sample.pose,1);
    float weight = 0;
    for(auto ele = tempScan.begin(); ele != tempScan.end(); ele++){
        if(ele->range < 5.0f){
            double endX = ele->origin.x + ele->range*cos(ele->theta);
            double endY = ele->origin.y + ele->range*sin(ele->theta);
            Point<double> startPoint = {ele->origin.x, ele->origin.y};
            Point<double> endPoint = {endX, endY};
            Point<int> startGrid = global_position_to_grid_cell(startPoint, map);
            Point<int> endGrid = global_position_to_grid_cell(endPoint, map);
            //printf("reached the 31th line tempScan size: %d\n", tempScan.size());
            //printf("reached the 32th line: %d\n", ii);
            ii++;
            if(map.isCellInGrid(endGrid.x, endGrid.y) && breshenham2(startGrid.x, startGrid.y, endGrid.x, endGrid.y, map)){
                // means that this sample particle is unlikely to be the right one, because the laser scan hits
                weight = weight - 12;
            }
            else{
                if(map.isCellInGrid(endGrid.x, endGrid.y) && map(endGrid.x, endGrid.y) > 60){
                    //means that this sample particle is likely
                    weight = weight -4;
                }
                else{
                    //end grid not in cell or map(endGrid) <= 60
                    weight = weight - 8;
                }
            }
        }
    }
    return weight/tempScan.size();
}

bool breshenham2(int xStart, int yStart, int xEnd, int yEnd, const OccupancyGrid& map){
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
            if(map(x, y) > 120){
                return true;//means hit something unex
            }
            if( p < 0){
                p += const1;
            }
            else{
                p += const2;
                y += dy;
            }
            x += dx;
        }
    }
    else{
        int p = 2 * Dx - Dy;
        int const1, const2;
        const1 = 2 * Dx;
        const2 = 2* (Dx - Dy);
        while( y != yEnd){
            if(map(x,y)>120){
                return true;
            }
            if( p < 0){
                p += const1;
            }
            else{
                p += const2;
                x += dx;
            }
            y += dy;
        }
    }
    return false;
}

