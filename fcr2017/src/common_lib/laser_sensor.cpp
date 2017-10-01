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

// void hokuyoCallBack(const sensor_msgs::LaserScan& msg)
// {
//     distanceFrontLeft = getSmallestLaserDistance(LASER_FRONT_READ_START, LASER_FRONT_READ_END, LASER_ARRAY_FRONT_READING_STEP, msg);
//     distanceFrontRight = getSmallestLaserDistance(-LASER_FRONT_READ_START, -LASER_FRONT_READ_END, LASER_ARRAY_FRONT_READING_STEP, msg);
//     distanceDiagonalLeft = getSmallestLaserDistance(LASER_DIAGONAL_READ_START, LASER_DIAGONAL_READ_END, LASER_ARRAY_FRONT_READING_STEP, msg);
//     distanceDiagonalRight = getSmallestLaserDistance(-LASER_DIAGONAL_READ_START, -LASER_DIAGONAL_READ_END, LASER_ARRAY_FRONT_READING_STEP, msg);

//     // ROS_INFO("distanceFrontLeft: %f", distanceFrontLeft);
//     // ROS_INFO("distanceFrontRight: %f", distanceFrontRight);
//     // ROS_INFO("distanceDiagonalLeft: %f", distanceDiagonalLeft);
//     // ROS_INFO("distanceDiagonalRight: %f", distanceDiagonalRight);
//     // ROS_INFO("distanceSideFront: %f", distanceSideFront);
//     // ROS_INFO("distanceSideMiddle: %f", distanceSideMiddle);
//     // ROS_INFO("distanceSideBack: %f", distanceSideBack);
// }
