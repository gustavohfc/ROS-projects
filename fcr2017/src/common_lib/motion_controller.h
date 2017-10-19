#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "common.h"
#include "odometer.h"


#define XY_GOAL_TOLERANCE 0.1 // metros
#define YAW_GOAL_TOLERANCE 0.1 // radianos
#define VELOCITY_TOLERANCE 0.0001 // m/s

#define MAX_LINEAR_VELOCITY 0.8 // m/s
#define MIN_LINEAR_VELOCITY 0.2 // m/s
#define MAX_ANGULAR_VELOCITY 0.7 // m/s
#define MIN_ANGULAR_VELOCITY 0.2 // m/s


class MotionController
{
private:
    std::vector<Position> goals;
    ros::Publisher pub_vel;
    PioneerState& current_state;
    const Odometer& odometer;

    void applyVelocityLimits(geometry_msgs::Twist& velocity);
    void updateState();
    double calculateLinearVelocity(double distance);
    double calculateAngularVelocity(double angle);
    void verifyObstacle();


public:
    MotionController(ros::NodeHandle& nodeHandle, PioneerState& _current_state, const Odometer& _odometer);

    void addGoal(Position new_goal);
    bool hasGoals();
    void goToGoal();
};


#endif // MOTION_CONTROLLER_H
