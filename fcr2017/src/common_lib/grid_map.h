#ifndef GRID_MAP_H
#define GRID_MAP_H

#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"


#define GRID_MAP_RESOLUTION 0.5

#define GRID_MAP_LIMIT_POSITIVE_X 46
#define GRID_MAP_LIMIT_NEGATIVE_X -37
#define GRID_MAP_LIMIT_POSITIVE_Y 26
#define GRID_MAP_LIMIT_NEGATIVE_Y -9

#define GRID_MAP_MATRIX_SIZE_X ( (int) ((GRID_MAP_LIMIT_POSITIVE_X - GRID_MAP_LIMIT_NEGATIVE_X) / GRID_MAP_RESOLUTION) )
#define GRID_MAP_MATRIX_SIZE_Y ( (int) ((GRID_MAP_LIMIT_POSITIVE_Y - GRID_MAP_LIMIT_NEGATIVE_Y) / GRID_MAP_RESOLUTION) )

#define GRID_MAP_MATRIX_OFFSET_X ( (int) ((-1 * GRID_MAP_LIMIT_NEGATIVE_X) / GRID_MAP_RESOLUTION) )
#define GRID_MAP_MATRIX_OFFSET_Y ( (int) ((-1 * GRID_MAP_LIMIT_NEGATIVE_Y) / GRID_MAP_RESOLUTION) )

#define EMPTY_CELL_VALUE 0
#define UNDEFINED_CELL_VALUE 50
#define OCCUPIED_CELL_VALUE 100

class GridMap
{
private:
    int8_t grid[GRID_MAP_MATRIX_SIZE_Y][GRID_MAP_MATRIX_SIZE_X];
    ros::Publisher pub;

    void sendMapToRviz();

public:
    void init(ros::NodeHandle& nodeHandle);
    void update();
};


// Global extern variable
extern GridMap gridMap;


#endif