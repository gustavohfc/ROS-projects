#ifndef GRID_MAP_H
#define GRID_MAP_H

#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"


#define GRID_MAP_RESOLUTION 0.1

#define GRID_MAP_LIMIT_POSITIVE_X 46
#define GRID_MAP_LIMIT_NEGATIVE_X -37
#define GRID_MAP_LIMIT_POSITIVE_Y 26
#define GRID_MAP_LIMIT_NEGATIVE_Y -9

#define GRID_MAP_MATRIX_SIZE_X ( (int) ((GRID_MAP_LIMIT_POSITIVE_X - GRID_MAP_LIMIT_NEGATIVE_X) / GRID_MAP_RESOLUTION) )
#define GRID_MAP_MATRIX_SIZE_Y ( (int) ((GRID_MAP_LIMIT_POSITIVE_Y - GRID_MAP_LIMIT_NEGATIVE_Y) / GRID_MAP_RESOLUTION) )

#define GRID_MAP_MATRIX_OFFSET_X ( (int) ((-1 * GRID_MAP_LIMIT_NEGATIVE_X) / GRID_MAP_RESOLUTION) )
#define GRID_MAP_MATRIX_OFFSET_Y ( (int) ((-1 * GRID_MAP_LIMIT_NEGATIVE_Y) / GRID_MAP_RESOLUTION) )

#define EMPTY_CELL 0
#define UNDEFINED_CELL 50
#define OCCUPIED_CELL 100

#define GRID_MAP_LASER_ANGLE_STEP 0.0174533 // ~1 grau
#define GRID_MAP_LASER_POINT_STEP (GRID_MAP_RESOLUTION / 2)
#define GRID_MAP_LASER_MAX_DISTANCE 5


class GridMap
{
private:
    int8_t grid[GRID_MAP_MATRIX_SIZE_Y][GRID_MAP_MATRIX_SIZE_X];
    ros::Publisher pub_rviz, pub_saver;

    void sendMapTo(ros::Publisher& pub);
    void setCell(double angle, double distance, int8_t newValue);

public:
    void init(ros::NodeHandle& nodeHandle);
    void update();
    void save();
};


// Global extern variable
extern GridMap gridMap;


#endif