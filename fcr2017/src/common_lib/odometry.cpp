#include "tf/transform_listener.h"

#include "odometry.h"

// Variaveis externas
Position pioneerPosition;
double currentLinearVelocity, currentAngularVelocity;

void poseCallBack(const nav_msgs::Odometry::ConstPtr& msg)
{
    pioneerPosition.x = msg->pose.pose.position.x;
    pioneerPosition.y = msg->pose.pose.position.y;

    tf::Pose tfPose;
    tf::poseMsgToTF(msg->pose.pose, tfPose);
    pioneerPosition.yaw = tf::getYaw(tfPose.getRotation());

    currentLinearVelocity = msg->twist.twist.linear.x;
    currentAngularVelocity = msg->twist.twist.angular.z;
}
