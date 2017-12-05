#ifndef USER_MOTION_CONTROLLER_H
#define USER_MOTION_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/Char.h"
#include "laser_sensor.h"

#define LINEAR_VELOCITY 0.6 // m/s
#define ANGULAR_VELOCITY 0.6 // m/s


class UserMotionController
{
private:
    const LaserSensor& laser_sensor;
    ros::Subscriber sub_controller;
    ros::Publisher pub_vel;
    char user_command;

public:
    UserMotionController(ros::NodeHandle& nodeHandle, const LaserSensor& _laser_sensor);

    bool resetReceived;

    void callback(const std_msgs::Char::ConstPtr& msg);
    void move();
};


#endif // USER_MOTION_CONTROLLER_H
