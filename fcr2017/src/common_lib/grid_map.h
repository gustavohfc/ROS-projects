#ifndef GRID_MAP_H
#define GRID_MAP_H

#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include "odometer.h"
#include "laser_sensor.h"
#include "common.h"


#define GRID_MAP_RESOLUTION 0.1

#define GRID_MAP_LIMIT_POSITIVE_X 46
#define GRID_MAP_LIMIT_NEGATIVE_X -37
#define GRID_MAP_LIMIT_POSITIVE_Y 26
#define GRID_MAP_LIMIT_NEGATIVE_Y -9

#define GRID_MAP_MATRIX_OFFSET_X ( (int) ((-1 * GRID_MAP_LIMIT_NEGATIVE_X) / GRID_MAP_RESOLUTION) )
#define GRID_MAP_MATRIX_OFFSET_Y ( (int) ((-1 * GRID_MAP_LIMIT_NEGATIVE_Y) / GRID_MAP_RESOLUTION) )

#define EMPTY_CELL 100
#define UNDEFINED_CELL 50
#define OCCUPIED_CELL 0

#define GRID_MAP_LASER_ANGLE_STEP 0.0174533 // ~1 grau
#define GRID_MAP_DISTANCE_STEP (GRID_MAP_RESOLUTION / 2)
#define GRID_MAP_LASER_MAX_DISTANCE 5


class GridMap
{
private:
    const Odometer& odometer;
    const LaserSensor& laser_sensor;
    const char ID;
    const double grid_x, grid_y;
    const int size_x, size_y;

    ros::Publisher pub_rviz;
    ros::Publisher pub_saver;

    std::string image_file_path;
    int8_t **grid;
    // int8_t grid[GRID_MAP_MATRIX_SIZE_Y][GRID_MAP_MATRIX_SIZE_X];


public:
    GridMap(char _ID, double _center_x, double _center_y, double _tolerance_x, double _tolerance_y, ros::NodeHandle& nodeHandle, const Odometer& _odometer, const LaserSensor& _laser_sensor, const char *fcr2017_path);
    ~GridMap();
    void saveImageFile();
    void updateGrid();
    bool markCoordinateAs(double coordinate_x, double coordinate_y, int8_t value);
    void sendToRviz();


// private:
//     int8_t grid[GRID_MAP_MATRIX_SIZE_Y][GRID_MAP_MATRIX_SIZE_X];
//     ros::Publisher pub_rviz, pub_saver;

//     void sendMapTo(ros::Publisher& pub);
//     void setCell(double angle, double distance, int8_t newValue);

// public:
// GridMap();

//     void init(ros::NodeHandle& nodeHandle);
//     void update();
//     void save();
};


#endif