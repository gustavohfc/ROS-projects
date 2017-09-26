#ifndef COMMON_H
#define COMMON_H

#include <deque>

#define XY_GOAL_TOLERANCE 0.2 // metros
#define YAW_GOAL_TOLERANCE 0.05 // radianos

#define MIN_OBSTACLE_DISTANCE 0.2
#define OBSTACLE_DISTACE_START_AVOIDANCE 0.5

#define MAX_LINEAR_VELOCITY 1
#define MIN_LINEAR_VELOCITY 0.2
#define MAX_ANGULAR_VELOCITY 0.7
#define MIN_ANGULAR_VELOCITY 0.2


struct Position
{
    double x, y, yaw;
    bool hasYaw;
    Position() : x(0), y(0), yaw(0), hasYaw(false) {}
    Position(double _x, double _y) : x(_x), y(_y), hasYaw(false) {}
    Position(double _x, double _y, double _yaw) : x(_x), y(_y), yaw(_yaw), hasYaw(true) {}
};

// Converte graus para radianos
#define degreesToRadians(angleDegrees) ((angleDegrees) * M_PI / 180.0)

enum States { WaitingForGoal, GoingToPoint, Bug2ModeLeft, Bug2ModeRight };
extern States pioneerState;

// Prototipos
void updateGoals(std::deque<Position>& goals);
void goToPosition(Position goal, ros::Publisher pub_cmd_vel);
double calculateLinearVelocity(double distance);
double calculateAngularVelocity(double angle);
double normalizeAngle(double angle);

#endif
