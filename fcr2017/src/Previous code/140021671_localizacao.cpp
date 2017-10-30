#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"
#include <math.h>

#define PI 3.14159265359

#define XY_GOAL_TOLERANCE 0.1
#define YAW_GOAL_TOLERANCE 0.1

#define MIN_LINEAR_SPEED 0.2
#define MAX_LINEAR_SPEED 1
#define MIN_ANGULAR_SPEED 0.2
#define MAX_ANGULAR_SPEED 0.7

struct position {double x; double y; double yaw;};

int currentGoal = 0;

const position goals[] =  { {5,   0,    1.57},
                            {5,   5,    3.14},
                            {0,   5,    -1.57},
                            {0,   0,    0},
                            };

position currentPosition = {0, 0, 0};
double currentLinearSpeed = 0;
double currentAngularSpeed = 0;


void poseCallBack(const nav_msgs::Odometry::ConstPtr& msg)
{
    currentPosition.x = msg->pose.pose.position.x;
    currentPosition.y = msg->pose.pose.position.y;

    tf::Pose tfPose;
    tf::poseMsgToTF(msg->pose.pose, tfPose);
    currentPosition.yaw = tf::getYaw(tfPose.getRotation());

    currentLinearSpeed = msg->twist.twist.linear.x;
    currentAngularSpeed = msg->twist.twist.angular.z;
}

bool is_linearSpeedZero()
{
    return fabs(currentLinearSpeed) <  0.0001;
}


bool is_angularSpeedZero()
{
    return fabs(currentAngularSpeed) <  0.0001;
}


bool is_xyReached()
{
    return fabs(currentPosition.x - goals[currentGoal].x) <  XY_GOAL_TOLERANCE && fabs(currentPosition.y - goals[currentGoal].y) <  XY_GOAL_TOLERANCE;
}


bool is_yawReached()
{
    return fabs(currentPosition.yaw - goals[currentGoal].yaw) <  YAW_GOAL_TOLERANCE;
}


bool is_goalReached()
{
    return is_linearSpeedZero() && is_angularSpeedZero() && is_xyReached() && is_yawReached();
}


double calculateTrajectoryLinearSpeed(double goalDistance, double goalRelativeTheta)
{
    if (fabs(goalRelativeTheta) > 0.785398)
        return 0;

    double speed =  MAX_LINEAR_SPEED;

    if (goalDistance < 2.5)
        speed -= MAX_LINEAR_SPEED * (1 - goalDistance / 2.5);

    // speed -= MAX_LINEAR_SPEED * fabs(goalRelativeTheta) / 0.785398;

    // ROS_INFO("Linear speed: %f", speed);

    if (speed > MAX_LINEAR_SPEED)
    {
        ROS_INFO("Linear speed too high");
        return MAX_LINEAR_SPEED;
    }
    if (speed < MIN_LINEAR_SPEED)
    {
        return MIN_LINEAR_SPEED;
    }
    else
    {
        return speed;
    }
}


double calculateTrajectoryAngularSpeed(double theta)
{
    double speed = MAX_ANGULAR_SPEED;

    int sinal = (theta > 0) ? 1 : -1;

    if (theta > 0 && theta > PI - 0.1)
    {
        theta = PI - theta;
        sinal *= -1;
    }
    else if (theta < 0 && theta < -PI + 0.1)
    {
        theta = PI + theta;
        sinal *= -1;
    }

    if (fabs(theta) < 1.57)
        speed -= MAX_ANGULAR_SPEED * (1 - fabs(theta) / 1.57);

    // ROS_INFO("Angular speed: %f /// %f /// %f", sinal * speed, theta, currentPosition.yaw);

    if (speed > MAX_ANGULAR_SPEED)
    {
        ROS_INFO("Angular speed too high");
        return sinal * MAX_ANGULAR_SPEED;
    }
    if (speed < MIN_ANGULAR_SPEED)
    {
        return sinal * MIN_ANGULAR_SPEED;
    }
    else
    {
        return sinal * speed;
    }
}


geometry_msgs::Twist calculateTrajectorySpeed()
{
    geometry_msgs::Twist vel;
    double goalDistanceX, goalDistanceY, goalDistance, goalRelativeTheta;


    goalDistanceX = goals[currentGoal].x - currentPosition.x;
    goalDistanceY = goals[currentGoal].y - currentPosition.y;
    goalDistance = hypot(goalDistanceX, goalDistanceY);
    goalRelativeTheta = atan2(goalDistanceY, goalDistanceX) - currentPosition.yaw;

    // ROS_INFO("Theta = %f ... %f ... %f", goalRelativeTheta, currentPosition.yaw, atan2(goalDistanceY, goalDistanceX));

    if (is_xyReached())
    {
        vel.linear.x = 0;
        if (is_yawReached())
        {
            vel.angular.z = 0;
        }
        else
        {
            vel.angular.z = calculateTrajectoryAngularSpeed(goals[currentGoal].yaw - currentPosition.yaw);
        }
    }
    else
    {
        vel.linear.x = calculateTrajectoryLinearSpeed(goalDistance, goalRelativeTheta);
        vel.angular.z = calculateTrajectoryAngularSpeed(goalRelativeTheta);
    }

    return vel;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "localizacao");
    ros::NodeHandle n;

    ros::Publisher pub_cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Subscriber sub_pose = n.subscribe("pose", 1, poseCallBack);

    ros::Rate loop_rate(10);

    int nGoals = sizeof(goals) / sizeof(*goals);

    // Imprime informacoes da simulacao
    ROS_INFO("A tolerancia para as cordenadas X e Y e' de %.2f metros.", XY_GOAL_TOLERANCE);
    ROS_INFO("A tolerancia para o angulo e' de %.2f radianos.", YAW_GOAL_TOLERANCE);
    for (int i = 0; i < nGoals; i++)
        ROS_INFO("Objetivo %d de %d: (%.2f, %.2f, %.2f)", i + 1, nGoals, goals[i].x, goals[i].y, goals[i].yaw);

    while (ros::ok())
    {
        ros::spinOnce();

        geometry_msgs::Twist vel_msg = calculateTrajectorySpeed();
        pub_cmd_vel.publish(vel_msg);

        if (is_goalReached())
        {
            ROS_INFO("Objetivo conlcuido (%.2f, %.2f, %.2f)", currentPosition.x, currentPosition.y, currentPosition.yaw);
            currentGoal++;
        }

        // Verifica se ja' concluiu todos os objetivos
        if (currentGoal == nGoals)
            break;

        loop_rate.sleep();
    }

    return 0;
}
