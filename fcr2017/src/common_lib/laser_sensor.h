#ifndef LASER_SENSOR_H
#define LASER_SENSOR_H

#include <sensor_msgs/LaserScan.h>


class LaserSensor
{
private:
	ros::Subscriber subscriber;

public:
    sensor_msgs::LaserScan::ConstPtr msg;

    LaserSensor(ros::NodeHandle& nodeHandle);
    void callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    double atAngle(double angle) const;
    double getAngleMin() const;
    double getAngleMax() const;
};

#endif
