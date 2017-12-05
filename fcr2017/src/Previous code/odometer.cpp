#include <tf/transform_listener.h>
#include <cmath>

#include "odometer.h"


Odometer::Odometer(ros::NodeHandle& nodeHandle)
{
    sub_odom = nodeHandle.subscribe("pose", 1, &Odometer::callback, this);
}


void Odometer::callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    this->msg = msg;

    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;

    tf::Pose tfPose;
    tf::poseMsgToTF(msg->pose.pose, tfPose);
    yaw = tf::getYaw(tfPose.getRotation());

    laserSensorX = x + (LASER_SENSOR_DISTANCE_FROM_CENTER * cos(yaw));
    laserSensorY = y + (LASER_SENSOR_DISTANCE_FROM_CENTER * sin(yaw));

    linearVelocity = msg->twist.twist.linear.x;
    angularVelocity = msg->twist.twist.angular.z;
}