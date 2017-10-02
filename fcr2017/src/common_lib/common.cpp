#include <ros/ros.h>

#include "common.h"
#include "odometer.h"

// Variaveis externas
States pioneerState = WaitingForGoal;

bool is_linearSpeedZero()               { return fabs(odometer.getLinearVelocity()) <  0.0001; }
bool is_angularSpeedZero()              { return fabs(odometer.getAngularVelocity()) <  0.0001; }
bool is_xyReached(double x, double y)   { return fabs(odometer.getX() - x) <  XY_GOAL_TOLERANCE && fabs(odometer.getY() - y) <  XY_GOAL_TOLERANCE; }
bool is_yawReached(double yaw)          { return fabs(odometer.getYaw() - yaw) <  YAW_GOAL_TOLERANCE; }
bool is_goalReached(Position p)         { return is_linearSpeedZero() && is_angularSpeedZero() && is_xyReached(p.x, p.y) && (is_yawReached(p.yaw) || !p.hasYaw); }


void updateGoals(std::deque<Position>& goals)
{
    if (is_goalReached(goals.front()))
    {
        ROS_INFO("Chegou na posicao (%.2f, %.2f, %.2f).", odometer.getX(), odometer.getY(), odometer.getYaw());
        goals.pop_front();

        if (!goals.empty())
            ROS_INFO("Indo para a posicao (%.2f, %.2f, %.2f).", goals.front().x, goals.front().y, goals.front().yaw);
    }
}


void goToPosition(Position goal, ros::Publisher pub_cmd_vel)
{
    geometry_msgs::Twist vel;

    if (is_xyReached(goal.x, goal.y))
    {
        vel.linear.x = 0;
        if (is_yawReached(goal.yaw) || !goal.hasYaw)
        {
            vel.angular.z = 0; // Chegou no objetivo
        }
        else
        {
            // Corrige o angulo final
            vel.angular.z = calculateAngularVelocity(goal.yaw - odometer.getYaw());
        }
    }
    else
    {
        // Calcula a distancia em relacao o objetivo
        double goalDistanceX = goal.x - odometer.getX();
        double goalDistanceY = goal.y - odometer.getY();
        double goalDistance = hypot(goalDistanceX, goalDistanceY);
        double goalAngle = normalizeAngle(atan2(goalDistanceY, goalDistanceX) - odometer.getYaw());

        vel.linear.x = calculateLinearVelocity(goalDistance);
        vel.angular.z = calculateAngularVelocity(goalAngle);

        // Reduz a velocidade linear nas curvas
        vel.linear.x *= 1 - (fabs(vel.angular.z) / MAX_ANGULAR_VELOCITY);
    }

    ROS_DEBUG("Linear: %.2f\t\tAngular: %.2f", vel.linear.x, vel.angular.z);

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