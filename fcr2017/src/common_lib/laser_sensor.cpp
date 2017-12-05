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



// Returns the closest reading of a specific angle
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


// Returns the minimun angle read by the sensor
double LaserSensor::getAngleMin() const
{
    return msg->angle_min;
}


// Returns the maximus angle read by the sensor
double LaserSensor::getAngleMax() const
{
    return msg->angle_max;
}

double LaserSensor::getWidth() const
{
    double width = 10000;

    for (double angle = getAngleMin(); angle < 0; angle += 0.0174533)
    {
        if (width > atAngle(angle) + atAngle(-1 * angle))
        {
            width = atAngle(angle) + atAngle(-1 * angle);
        }
    }

    return width;
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

