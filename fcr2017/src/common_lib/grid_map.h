#ifndef GRID_MAP_H
#define GRID_MAP_H

#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include "odometer.h"
#include "laser_sensor.h"
#include "common.h"


#define GRID_MAP_RESOLUTION 0.01

#define EMPTY_CELL 0
#define UNDEFINED_CELL 50
#define OCCUPIED_CELL 100

#define GRID_MAP_LASER_ANGLE_STEP 0.0174533 // ~1 grau
#define GRID_MAP_DISTANCE_STEP (GRID_MAP_RESOLUTION / 2)
#define GRID_MAP_LASER_MAX_DISTANCE 3.5


class GridMap
{
private:
    const Odometer& odometer;
    const LaserSensor& laser_sensor;
    const char ID;
    const double grid_x, grid_y;
    const int size_x, size_y;

    ros::Publisher pub_rviz;

    std::string image_file_path;
    int8_t **grid;

public:
    GridMap(char _ID, double _center_x, double _center_y, double _tolerance_x, double _tolerance_y, ros::NodeHandle& nodeHandle, const Odometer& _odometer, const LaserSensor& _laser_sensor, const char *fcr2017_path);
    ~GridMap();
    void saveImageFile();
    void updateGrid();
    bool markCoordinateAs(double coordinate_x, double coordinate_y, int8_t value);
    void sendToRviz();
};


#endif