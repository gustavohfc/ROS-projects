#include "user_motion_controller.h"
#include "common.h"


UserMotionController::UserMotionController(ros::NodeHandle& nodeHandle, const LaserSensor& _laser_sensor)
    : laser_sensor(_laser_sensor), user_command(0), resetReceived(false)
{
    sub_controller = nodeHandle.subscribe("controller", 1, &UserMotionController::callback, this);
    pub_vel = nodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}


void UserMotionController::callback(const std_msgs::Char::ConstPtr& msg)
{
    user_command = msg->data;

    if (user_command == 'r')
    {
        resetReceived = true;
    }
}


void UserMotionController::move()
{
    geometry_msgs::Twist velocity;

    // Check if there are obstacles in the front
    bool frontClear =  1 < laser_sensor.getShortestDistance(degreesToRadians(-10), degreesToRadians(10));

    // Check if the user wants the Pioneer to go forward
    if ((user_command == 'q' || user_command == 'w' || user_command == 'e') && frontClear)
    {
        velocity.linear.x = LINEAR_VELOCITY;
    }
    else
    {
        velocity.linear.x = 0;
    }

    // Check if the user wants the Pioneer to turn
    if (user_command == 'q' || user_command == 'a' || user_command == 'e' || user_command == 'd')
    {
        int direction = (user_command == 'q' || user_command == 'a') ? 1 : -1;
        velocity.angular.z = direction * LINEAR_VELOCITY;
    }
    else
    {
        velocity.angular.z = 0;
    }

    pub_vel.publish(velocity);
}