#include <ros/ros.h>
#include <limits>

#include "laser_sensor.h"

LaserSensor::LaserSensor(ros::NodeHandle& nodeHandle)
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
    try
    {
        return msg->ranges.at((msg->ranges.size() / 2) + (angle / msg->angle_increment));
    }
    catch (const std::out_of_range& oor)
    {
        ROS_INFO("Angle out of range at method LaserSensor::atAngle(), angle = %f", angle);
        return 0;
    }
}



double LaserSensor::getAngleMin() const
{
    return msg->angle_min;
}



double LaserSensor::getAngleMax() const
{
    return msg->angle_max;
}



// Returns the smallest distance read by the laser sensor in the interval
double LaserSensor::getShortestDistance(double angle1, double angle2) const
{
    int first_index, last_index;
    double shortestDistance = std::numeric_limits<double>::max();

    if (angle1 < angle2)
    {
        first_index = (msg->ranges.size() / 2) + (angle1 / msg->angle_increment);
        last_index = (msg->ranges.size() / 2) + (angle2 / msg->angle_increment);
    }
    else
    {
        first_index = (msg->ranges.size() / 2) + (angle2 / msg->angle_increment);
        last_index = (msg->ranges.size() / 2) + (angle1 / msg->angle_increment);
    }

    if (first_index < 0)
        first_index = 0;

    if (last_index > (msg->ranges.size() - 1))
        last_index = msg->ranges.size() - 1;

    try
    {
        for (int i = first_index; i < last_index; i++)
            if (msg->ranges.at(i) < shortestDistance)
                shortestDistance = msg->ranges.at(i);
    }
    catch (const std::out_of_range& oor)
    {
        ROS_INFO("Angle out of range at method LaserSensor::getShortestDistance()");
        return 0;
    }

    return shortestDistance;
}