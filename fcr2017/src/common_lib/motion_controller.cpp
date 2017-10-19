#include "motion_controller.h"

inline bool is_velocity_zero(double velocity)   { return ((velocity < 0) ? -velocity : velocity) < VELOCITY_TOLERANCE; }
inline bool is_coordinate_reached(double c)     { return ((c < 0) ? -c : c) < XY_GOAL_TOLERANCE; }
inline bool is_yaw_reached(double yaw)          { return ((yaw < 0) ? -yaw : yaw) < YAW_GOAL_TOLERANCE; }



MotionController::MotionController(ros::NodeHandle& nodeHandle, PioneerState& _current_state, const Odometer& _odometer)
    : odometer(_odometer), current_state(_current_state)
{
    // Initialize topic publisher
    pub_vel = nodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}



void MotionController::addGoal(Position new_goal)
{
    goals.push_back(new_goal);
}



bool MotionController::hasGoals()
{
    return !goals.empty();
}



void MotionController::goToGoal()
{
    geometry_msgs::Twist velocity;

    updateState();

    // Calculate distance to goal
    double goalDistanceX = goals.front().x - odometer.getX();
    double goalDistanceY = goals.front().y - odometer.getY();
    double goalDistance = hypot(goalDistanceX, goalDistanceY);
    double goalAngle = normalizeAngle(atan2(goalDistanceY, goalDistanceX) - odometer.getYaw());

    switch(current_state.state)
    {
        case GoingToXY:
            velocity.linear.x = calculateLinearVelocity(goalDistance);
            velocity.angular.z = calculateAngularVelocity(goalAngle);

            //Reduz a velocidade linear nas curvas
            velocity.linear.x *= 1 - (fabs(velocity.angular.z) / MAX_ANGULAR_VELOCITY);
            break;

        case GoingToYaw:
            velocity.linear.x = 0;
            velocity.angular.z = calculateAngularVelocity(goals.front().yaw - odometer.getYaw());
            break;

        case AvoidingObstacle:
            break;

        case GoalReached:
            velocity.linear.x = 0;
            velocity.angular.z = 0;

            ROS_INFO("Chegou na posiao (%f, %f, %f)", odometer.getX(), odometer.getY(), odometer.getYaw());

            goals.erase(goals.begin());

            if(hasGoals())
            {
                ROS_INFO("Indo para a posiao (%f, %f, %f)", goals.front().x, goals.front().y, goals.front().yaw);
                current_state.state = GoingToXY;
            }

            break;

        default:
            ROS_DEBUG("Default case on MotionController::goToGoal()");
            break;
    }


    ROS_DEBUG("Linear: %f\t\tAngular:%f", velocity.linear.x, velocity.angular.z);
    pub_vel.publish(velocity);
}



double MotionController::calculateLinearVelocity(double distance)
{
    double linearVelocity =  MAX_LINEAR_VELOCITY;

    // Reduz a velocidade quando está chegando no objetivo
    if (distance < 2.5)
        linearVelocity -= MAX_LINEAR_VELOCITY * (1 - distance / 2.5);

    // Limit the linear velocity
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



double MotionController::calculateAngularVelocity(double angle)
{
    angle = normalizeAngle(angle);

    if (fabs(angle) < YAW_GOAL_TOLERANCE / 2)
        return 0;

    double angularVelocity = MAX_ANGULAR_VELOCITY;
    int direcao = (angle > 0) ? 1 : -1;

    if (fabs(angle) < M_PI_2)
        angularVelocity -= MAX_ANGULAR_VELOCITY * (1 - fabs(angle) / M_PI_2);

    // Limit the angular velocity
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



void MotionController::updateState()
{
    switch(current_state.state)
    {
        case GoingToXY:
            if (is_coordinate_reached(goals.front().x - odometer.getX()) && is_coordinate_reached(goals.front().y - odometer.getY()))
            {
                if (goals.front().hasYaw)
                    current_state.state = GoingToYaw;
                else
                    current_state.state = GoalReached;
            }
            break;

        case GoingToYaw:
            if (!goals.front().hasYaw || is_yaw_reached(goals.front().yaw - odometer.getYaw()))
                current_state.state = GoalReached;
            break;

        case AvoidingObstacle:
            break;

        default:
            ROS_DEBUG("Default case on MotionController::updateState()");
            break;
    }
}



void MotionController::verifyObstacle()
{}



void MotionController::applyVelocityLimits(geometry_msgs::Twist& velocity)
{
}