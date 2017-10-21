#ifndef COMMON_H
#define COMMON_H


// Converte graus para radianos
#define degreesToRadians(angleDegrees) ((angleDegrees) * M_PI / 180.0)


struct Position
{
    double x, y, yaw;
    bool hasYaw;
    Position() : x(0), y(0), yaw(0), hasYaw(false) {}
    Position(double _x, double _y) : x(_x), y(_y), yaw(0), hasYaw(false) {}
    Position(double _x, double _y, double _yaw) : x(_x), y(_y), yaw(_yaw), hasYaw(true) {}
};


enum PioneerPossibleState { GoingToXY, GoingToYaw, AvoidingObstacle, GoalReached };
struct PioneerState
{
    PioneerState(PioneerPossibleState start_state) : state(start_state) {}

    PioneerPossibleState state;

    // Stores specific information for some states
    union
    {
        struct
        {
            double goal_distance, goal_angle;
            double front_left_object_distance, wide_left_object_distance;
            double front_right_object_distance, wide_right_object_distance;
            double goal_object_distance, goal_perpendicular_object_distance;
            PioneerPossibleState previous_state; // Used to restore the previous state
        } AvoidingObstacleInfo;
    };
};


double normalizeAngle(double angle);

#endif
