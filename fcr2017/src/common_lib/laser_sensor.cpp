#include <ros/ros.h>

#include "laser_sensor.h"

// Global extern variable
LaserSensor laserSensor;


void LaserSensor::init(ros::NodeHandle& nodeHandle, ros::Subscriber& subscriber)
{
    subscriber = nodeHandle.subscribe("hokuyo_scan", 1, &LaserSensor::callback, this);
}


void LaserSensor::callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    this->msg = msg;
}


// Retorna a leitura de distancia mais proxima do angulo (em radianos)
double LaserSensor::atAngle(double angle) const
{
    return msg->ranges[ (msg->ranges.size() / 2) + (angle / msg->angle_increment) ];
}


double LaserSensor::getAngleMin() const
{
    return msg->angle_min;
}


double LaserSensor::getAngleMax() const
{
    return msg->angle_max;
}
