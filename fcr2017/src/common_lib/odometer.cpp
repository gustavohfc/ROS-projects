#include "tf/transform_listener.h"

#include "odometer.h"

// Global extern variable
Odometer odometer;


void Odometer::init(ros::NodeHandle& nodeHandle, ros::Subscriber subscriber)
{
    subscriber = nodeHandle.subscribe("pose", 1, &Odometer::callback, this);
}


void Odometer::callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    this->msg = msg;

    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;

    tf::Pose tfPose;
    tf::poseMsgToTF(msg->pose.pose, tfPose);
    yaw = tf::getYaw(tfPose.getRotation());

    linearVelocity = msg->twist.twist.linear.x;
    angularVelocity = msg->twist.twist.angular.z;
}