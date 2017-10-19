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

	union
	{
	} state_info;
};


double normalizeAngle(double angle);

#endif
