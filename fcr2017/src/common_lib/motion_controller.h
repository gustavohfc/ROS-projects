#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <vector>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "common.h"
#include "odometer.h"
#include "laser_sensor.h"


#define XY_GOAL_TOLERANCE 0.1 // metros
#define YAW_GOAL_TOLERANCE 0.1 // radianos
#define VELOCITY_TOLERANCE 0.0001 // m/s

#define MAX_LINEAR_VELOCITY 0.4 // m/s
#define MIN_LINEAR_VELOCITY 0.2 // m/s
#define MAX_ANGULAR_VELOCITY 0.4 // m/s
#define MIN_ANGULAR_VELOCITY 0.2 // m/s

#define FRONT_LASER_ANGLE 30
#define GOAL_LASER_ANGLE 1.0

#define START_OBSTACLE_AVOIDING_DISTANCE 1
#define OBSTACLE_AVOIDING_AVERAGE_DISTANCE 0.7
#define OBSTACLE_BUG_TOLERANCE 0.05
#define OBSTACLE_CRITIAL_DISTANCE 0.4


class MotionController
{
private:
    std::vector<Position> goals;
    ros::Publisher pub_vel;
    PioneerState& current_state;
    const Odometer& odometer;
    const LaserSensor& laser_sensor;

    void updateState();
    double calculateLinearVelocity(double distance);
    double calculateAngularVelocity(double angle);
    void checkObstacles(double goal_distance, double goal_angle);
    void calculateVelocityAvoidingObstacle(geometry_msgs::Twist& velocity);


public:
    MotionController(ros::NodeHandle& nodeHandle, PioneerState& _current_state, const Odometer& _odometer, const LaserSensor& _laser_sensor);

    void addGoal(Position new_goal);
    void addGoals(std::vector<Position> new_goals);
    bool hasGoals();
    void goToGoal();
    std::string getGoalsString();
};


#endif // MOTION_CONTROLLER_H
