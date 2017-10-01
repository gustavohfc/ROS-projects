#ifndef LASER_SENSOR_H
#define LASER_SENSOR_H

#include <sensor_msgs/LaserScan.h>


class LaserSensor
{
private:


public:
    sensor_msgs::LaserScan::ConstPtr msg;

    void init(ros::NodeHandle& nodeHandle, ros::Subscriber& subscriber);
    void callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    double atAngle(double angle) const;
    double getAngleMin() const;
    double getAngleMax() const;
};


// Global extern variable
extern LaserSensor laserSensor;


#endif
