#ifndef LASER_SENSOR_H
#define LASER_SENSOR_H

#include <sensor_msgs/LaserScan.h>

class LaserSensor
{
private:
    sensor_msgs::LaserScan::ConstPtr msg;


public:
    void init(ros::NodeHandle& nodeHandle, ros::Subscriber subscriber);
    void callback(const sensor_msgs::LaserScan::ConstPtr& msg);
};


// Global extern variable
extern LaserSensor laserSensor;


#endif
