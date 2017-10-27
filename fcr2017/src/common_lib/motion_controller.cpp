#include <algorithm>

#include "motion_controller.h"

inline bool is_velocity_zero(double velocity)   { return ((velocity < 0) ? -velocity : velocity) < VELOCITY_TOLERANCE; }
inline bool is_coordinate_reached(double c)     { return ((c < 0) ? -c : c) < XY_GOAL_TOLERANCE; }
inline bool is_yaw_reached(double yaw)          { return ((yaw < 0) ? -yaw : yaw) < YAW_GOAL_TOLERANCE; }



MotionController::MotionController(ros::NodeHandle& nodeHandle, PioneerState& _current_state, const Odometer& _odometer, const LaserSensor& _laser_sensor)
    : odometer(_odometer), current_state(_current_state), laser_sensor(_laser_sensor)
{
    // Initialize topic publisher
    pub_vel = nodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}



void MotionController::addGoal(Position new_goal)
{
    goals.push_back(new_goal);
}



void MotionController::addGoals(std::vector<Position> new_goals)
{
    goals.insert(goals.end(), new_goals.begin(), new_goals.end());
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
    double goal_distance_x = goals.front().x - odometer.getX();
    double goal_distance_y = goals.front().y - odometer.getY();
    double goal_distance = hypot(goal_distance_x, goal_distance_y);
    double goal_angle = normalizeAngle(atan2(goal_distance_y, goal_distance_x) - odometer.getYaw());

    checkObstacles(goal_distance, goal_angle);

    switch(current_state.state)
    {
        case GoingToXY:
            velocity.linear.x = calculateLinearVelocity(goal_distance);
            velocity.angular.z = calculateAngularVelocity(goal_angle);

            //Reduz a velocidade linear nas curvas
            velocity.linear.x *= 1 - (fabs(velocity.angular.z) / MAX_ANGULAR_VELOCITY);
            break;

        case GoingToYaw:
            velocity.linear.x = 0;
            velocity.angular.z = calculateAngularVelocity(goals.front().yaw - odometer.getYaw());
            break;

        case AvoidingObstacle:
            calculateVelocityAvoidingObstacle(velocity);
            break;

        case GoalReached:
            velocity.linear.x = 0;
            velocity.angular.z = 0;

            if (goals[0].hasNode_ID)
                ROS_INFO("Chegou na posiao (%f, %f, %f) - No %c", odometer.getX(), odometer.getY(), odometer.getYaw(), goals[0].node_ID);
            else
                ROS_INFO("Chegou na posiao (%f, %f, %f)", odometer.getX(), odometer.getY(), odometer.getYaw());

            goals.erase(goals.begin());

            if(hasGoals())
            {
                ROS_INFO("Proximos objetivos: [ %s ]", getGoalsString().c_str());
                current_state.state = GoingToXY;
            }
            else
                current_state.state = GoalReached;

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

    if (fabs(angle) < M_PI_4)
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



void MotionController::checkObstacles(double goal_distance, double goal_angle)
{
    double front_left_object_distance = laser_sensor.getShortestDistance(0, degreesToRadians(FRONT_LASER_ANGLE));

    double wide_left_object_distance = std::min(front_left_object_distance,
                                             laser_sensor.getShortestDistance(degreesToRadians(FRONT_LASER_ANGLE), degreesToRadians(90)));

    double front_right_object_distance = laser_sensor.getShortestDistance(0, degreesToRadians(-FRONT_LASER_ANGLE));

    double wide_right_object_distance = std::min(front_right_object_distance,
                                              laser_sensor.getShortestDistance(degreesToRadians(-FRONT_LASER_ANGLE), degreesToRadians(-90)));

    double goal_object_distance = laser_sensor.getShortestDistance(goal_angle - degreesToRadians(GOAL_LASER_ANGLE / 2),
                                                                 goal_angle + degreesToRadians(GOAL_LASER_ANGLE / 2));

    double goal_perpendicular_object_distance = fabs(sin(goal_angle) * goal_object_distance);

    // Check if it's necessary to avoid obstacle
    if (front_left_object_distance          < START_OBSTACLE_AVOIDING_DISTANCE                              ||
        front_right_object_distance         < START_OBSTACLE_AVOIDING_DISTANCE                              ||
        // goal_perpendicular_object_distance  < OBSTACLE_AVOIDING_AVERAGE_DISTANCE + OBSTACLE_BUG_TOLERANCE   ||
        wide_left_object_distance           < OBSTACLE_CRITIAL_DISTANCE                                     ||
        wide_right_object_distance          < OBSTACLE_CRITIAL_DISTANCE)
    {
        // Save information for the obstacle avoidance method
        current_state.AvoidingObstacleInfo.goal_distance = goal_distance;
        current_state.AvoidingObstacleInfo.goal_angle = goal_angle;
        current_state.AvoidingObstacleInfo.front_left_object_distance = front_left_object_distance;
        current_state.AvoidingObstacleInfo.wide_left_object_distance = wide_left_object_distance;
        current_state.AvoidingObstacleInfo.front_right_object_distance = front_right_object_distance;
        current_state.AvoidingObstacleInfo.wide_right_object_distance = wide_right_object_distance;
        current_state.AvoidingObstacleInfo.goal_object_distance = goal_object_distance;
        current_state.AvoidingObstacleInfo.goal_perpendicular_object_distance = goal_perpendicular_object_distance;

        if (current_state.state != AvoidingObstacle)
        {
            // Save previous state
            current_state.AvoidingObstacleInfo.previous_state = current_state.state;

            // Update state
            current_state.state = AvoidingObstacle;
        }
    }
    else
    {
        if (current_state.state == AvoidingObstacle)
        {
            // Stop the obstacle avoidance
            current_state.state = current_state.AvoidingObstacleInfo.previous_state;
        }
    }
}



void MotionController::calculateVelocityAvoidingObstacle(geometry_msgs::Twist& velocity)
{
    // Verify if the distance is critial
    if (current_state.AvoidingObstacleInfo.front_left_object_distance  < OBSTACLE_CRITIAL_DISTANCE ||
        current_state.AvoidingObstacleInfo.front_right_object_distance < OBSTACLE_CRITIAL_DISTANCE ||
        current_state.AvoidingObstacleInfo.wide_left_object_distance   < OBSTACLE_CRITIAL_DISTANCE ||
        current_state.AvoidingObstacleInfo.wide_right_object_distance  < OBSTACLE_CRITIAL_DISTANCE)
    {
        ROS_DEBUG("Critial object distance");

        if (current_state.AvoidingObstacleInfo.wide_left_object_distance > current_state.AvoidingObstacleInfo.wide_right_object_distance)
        {
            velocity.linear.x = 0;
            velocity.angular.z = MIN_ANGULAR_VELOCITY;
        }
        else
        {
            velocity.linear.x = 0;
            velocity.angular.z = -MIN_ANGULAR_VELOCITY;
        }
    }
    else
    {
        // If front is clear then do the Bug 0 algorithm and the goal angle is inside the laser sensor readings
        // if (current_state.AvoidingObstacleInfo.front_left_object_distance  > OBSTACLE_AVOIDING_AVERAGE_DISTANCE  &&
        //     current_state.AvoidingObstacleInfo.front_right_object_distance > OBSTACLE_AVOIDING_AVERAGE_DISTANCE  &&
        //     current_state.AvoidingObstacleInfo.goal_object_distance != std::numeric_limits<double>::max())
        // {
        //     ROS_DEBUG("Bug algorithm");

        //     velocity.linear.x = MIN_LINEAR_VELOCITY;

        //     if (current_state.AvoidingObstacleInfo.goal_perpendicular_object_distance < OBSTACLE_AVOIDING_AVERAGE_DISTANCE - OBSTACLE_BUG_TOLERANCE)
        //         velocity.angular.z = ((current_state.AvoidingObstacleInfo.goal_angle < 0) ? 1 : -1) * MIN_LINEAR_VELOCITY;

        //     else if (current_state.AvoidingObstacleInfo.goal_perpendicular_object_distance < OBSTACLE_AVOIDING_AVERAGE_DISTANCE - OBSTACLE_BUG_TOLERANCE)
        //         velocity.angular.z = ((current_state.AvoidingObstacleInfo.goal_angle < 0) ? -1 : 1) * MIN_LINEAR_VELOCITY;

        //     else
        //         velocity.angular.z = 0;
        // }

        // If only one side is blocked then go to the clear side
        /*else*/ if (current_state.AvoidingObstacleInfo.front_left_object_distance > START_OBSTACLE_AVOIDING_DISTANCE)
        {
            ROS_DEBUG("Turning left");
            velocity.linear.x = MIN_LINEAR_VELOCITY;
            velocity.angular.z = MIN_ANGULAR_VELOCITY;
        }
        else if(current_state.AvoidingObstacleInfo.front_right_object_distance > START_OBSTACLE_AVOIDING_DISTANCE)
        {
            ROS_DEBUG("Turning right");
            velocity.linear.x = MIN_LINEAR_VELOCITY;
            velocity.angular.z = -MIN_ANGULAR_VELOCITY;
        }

        // If both sides are blocked then stop and find the closest espace angle
        else
        {
            ROS_DEBUG("Front blocked");

            velocity.linear.x = 0;

            for (double angle = 0; angle < laser_sensor.getAngleMax(); angle += degreesToRadians(1))
            {
                if (laser_sensor.atAngle(angle) > START_OBSTACLE_AVOIDING_DISTANCE)
                {
                    velocity.angular.z = MIN_ANGULAR_VELOCITY;
                }
                else if (laser_sensor.atAngle(-angle) > START_OBSTACLE_AVOIDING_DISTANCE)
                {
                    velocity.angular.z = -MIN_ANGULAR_VELOCITY;
                }
            }
        }
    }
}



std::string MotionController::getGoalsString()
{
    std::string goals_string;

    for(std::vector<Position>::iterator it = goals.begin(); it != goals.end(); ++it)
    {
        if (it != goals.begin())
            goals_string += " -> ";

        if (it->hasNode_ID)
            goals_string += it->node_ID;
        else
            goals_string += "NOT IMPLEMENTED YET";
    }

    return goals_string;
}
