#include "ros/ros.h"
#include <deque>

#include "common.h"
#include "odometry.h"

// Variaveis externas
States pioneerState = WaitingForGoal;

bool is_linearSpeedZero()               { return fabs(currentLinearVelocity) <  0.0001; }
bool is_angularSpeedZero()              { return fabs(currentAngularVelocity) <  0.0001; }
bool is_xyReached(double x, double y)   { return fabs(pioneerPosition.x - x) <  XY_GOAL_TOLERANCE && fabs(pioneerPosition.y - y) <  XY_GOAL_TOLERANCE; }
bool is_yawReached(double yaw)          { return fabs(pioneerPosition.yaw - yaw) <  YAW_GOAL_TOLERANCE; }
bool is_goalReached(Position p)         { return is_linearSpeedZero() && is_angularSpeedZero() && is_xyReached(p.x, p.y) && is_yawReached(p.yaw); }


void updateGoals(std::deque<Position> goals)
{
    if (is_goalReached(goals.front()))
    {
        ROS_INFO("Chegou na posicao (%.2f, %.2f, %.2f).", pioneerPosition.x, pioneerPosition.y, pioneerPosition.yaw);
        goals.pop_front();
    }
}


void goToPosition(Position goal, ros::Publisher pub_cmd_vel)
{
    geometry_msgs::Twist vel;
    double distance, angle; // Informacoes sobre o proximo objetivo, podendo ser ir para o ponto ou desviar de um obstaculo

    // Calcula a distancia em relacao o objetivo
    double goalDistanceX = goal.x - pioneerPosition.x;
    double goalDistanceY = goal.y - pioneerPosition.y;
    distance = hypot(goalDistanceX, goalDistanceY);
    angle = normalizeAngle(atan2(goalDistanceY, goalDistanceX) - pioneerPosition.yaw);

    vel.linear.x = calculateLinearVelocity(distance);
    vel.angular.z = calculateAngularVelocity(angle);

    pub_cmd_vel.publish(vel);
}


double calculateLinearVelocity(double distance)
{
    double linearVelocity =  MAX_LINEAR_VELOCITY;

    // Reduz a velocidade quando está chegando no objetivo
    if (distance < 2.5)
        linearVelocity -= MAX_LINEAR_VELOCITY * (1 - distance / 2.5);

    if (linearVelocity > MAX_LINEAR_VELOCITY)
    {
        ROS_DEBUG("Linear velocity too high");
        return MAX_LINEAR_VELOCITY;
    }
    if (linearVelocity < MIN_LINEAR_VELOCITY)
    {
        return MIN_LINEAR_VELOCITY;
    }
    else
    {
        return linearVelocity;
    }
}


double calculateAngularVelocity(double angle)
{
    angle = normalizeAngle(angle);

    if (fabs(angle) < YAW_GOAL_TOLERANCE / 2)
        return 0;

    double angularVelocity = MAX_ANGULAR_VELOCITY;
    int direcao = (angle > 0) ? 1 : -1;

    if (fabs(angle) < M_PI_2)
        angularVelocity -= MAX_ANGULAR_VELOCITY * (1 - fabs(angle) / M_PI_2);

    if (angularVelocity > MAX_ANGULAR_VELOCITY)
    {
        ROS_DEBUG("Angular velocity too high");
        return direcao * MAX_ANGULAR_VELOCITY;
    }
    if (angularVelocity < MIN_ANGULAR_VELOCITY)
    {
        return direcao * MIN_ANGULAR_VELOCITY;
    }
    else
    {
        return direcao * angularVelocity;
    }
}


// Retorna o angulo equivalente entre PI e -PI
double normalizeAngle(double angle)
{
    angle = fmod(angle + M_PI, 2 * M_PI);
    if (angle < 0)
        angle += 2 * M_PI;

    return angle - M_PI;
}