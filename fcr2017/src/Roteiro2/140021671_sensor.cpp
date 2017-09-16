#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "p2os_msgs/SonarArray.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"
#include <limits>
#include <string>
#include <math.h>


// Seleciona o sensor utilizado
#define SONAR
//#define HOKUYO_SCAN

const double sonarAngle[] = { -1.5708, -0.872665, -0.523599, -0.174533, 0.174533, 0.523599, 0.872665, 1.5708};
ros::Publisher pub_cmd_vel;

void getPioneerAbsolutePosition(geometry_msgs::Pose2D &pose)
{
    std::string temp = std::string("/pose");
    boost::shared_ptr<nav_msgs::Odometry const> odometryPose = ros::topic::waitForMessage<nav_msgs::Odometry>(temp);

    pose.x = odometryPose->pose.pose.position.x;
    pose.y = odometryPose->pose.pose.position.y;

    tf::Pose tfPose;
    tf::poseMsgToTF(odometryPose->pose.pose, tfPose);
    pose.theta = tf::getYaw(tfPose.getRotation());

    // ROS_INFO("%f --- %f --- %f", pose.x, pose.y, pose.theta);
}

void getObstacleRelativePosition(geometry_msgs::Pose2D &relative, double distance, double referenceAngle, double sensorAngle)
{
    double theta = referenceAngle + sensorAngle;

    relative.x = cos(theta) * distance;
    relative.y = sin(theta) * distance;

    // ROS_INFO("%f --- %f", relative.x, relative.y);
}

void getObstacleAbsolutePosition(geometry_msgs::Pose2D &absolute, geometry_msgs::Pose2D pioneerAbsolutePosition, geometry_msgs::Pose2D obstacleRelativePosition)
{
    absolute.x = pioneerAbsolutePosition.x + obstacleRelativePosition.x;
    absolute.y = pioneerAbsolutePosition.y + obstacleRelativePosition.y;

    // ROS_INFO("%f --- %f", absolute.x, absolute.y);
}

void avoidObstacle()
{
    geometry_msgs::Twist vel;

    // Gira em 180 graus
    vel.linear.x = 0;
    vel.angular.z = 0.5;
    pub_cmd_vel.publish(vel);

    ros::Duration(6.28).sleep();

    vel.linear.x = 0;
    vel.angular.z = 0;
    pub_cmd_vel.publish(vel);
}



#ifdef SONAR

void sensorCallBack(const p2os_msgs::SonarArray& msg)
{
    int closestObstacleSensor = -1;
    double closestObstacleDistance = std::numeric_limits<double>::infinity();
    geometry_msgs::Twist vel;
    geometry_msgs::Pose2D pioneerAbsolutePosition, obstacleRelativePosition, obstacleAbsolutePosition;


    for (int i = 1; i < 7; i++)
    {
        if(msg.ranges[i] < closestObstacleDistance)
        {
            closestObstacleDistance = msg.ranges[i];
            closestObstacleSensor = i;
        }
    }

    if(closestObstacleSensor == -1)
    {
        // Continua andando, nao tem objetos na frente
        vel.linear.x = 0.5;
        vel.angular.z = 0;
        pub_cmd_vel.publish(vel);
    }
    else
    {
        // Para o pioneer, tem objeto no caminho
        vel.linear.x = 0;
        vel.angular.z = 0;
        pub_cmd_vel.publish(vel);

        getPioneerAbsolutePosition(pioneerAbsolutePosition);
        getObstacleRelativePosition(obstacleRelativePosition, closestObstacleDistance, pioneerAbsolutePosition.theta, sonarAngle[closestObstacleSensor]);
        getObstacleAbsolutePosition(obstacleAbsolutePosition, pioneerAbsolutePosition, obstacleRelativePosition);

        ROS_INFO("Objeto localizado pelo sensor %d na posicao absoluta (%.2f, %.2f), desviando do objeto.", closestObstacleSensor, obstacleAbsolutePosition.x, obstacleAbsolutePosition.y);

        avoidObstacle();
    }
}

#endif




#ifdef HOKUYO_SCAN

void sensorCallBack(const p2os_msgs::SonarArray& msg)
{
    int closestObstacleSensor = -1;
    double closestObstacleDistance = std::numeric_limits<double>::infinity();
    geometry_msgs::Twist vel;
    geometry_msgs::Pose2D pioneerAbsolutePosition, obstacleRelativePosition, obstacleAbsolutePosition;


    // for (int i = 1; i < 7; i++)
    // {
    //   if(msg.ranges[i] < closestObstacleDistance)
    //   {
    //     closestObstacleDistance = msg.ranges[i];
    //     closestObstacleSensor = i;
    //   }
    // }

    if(closestObstacleSensor == -1)
    {
        // Continua andando, nao tem objetos na frente
        vel.linear.x = 1;
        vel.angular.z = 0;
        pub_cmd_vel.publish(vel);
    }
    else
    {
        // Para o pioneer, tem objeto no caminho
        vel.linear.x = 0;
        vel.angular.z = 0;
        pub_cmd_vel.publish(vel);

        getPioneerAbsolutePosition(pioneerAbsolutePosition);
        getObstacleRelativePosition(obstacleRelativePosition, closestObstacleDistance, pioneerAbsolutePosition.theta, sonarAngle[closestObstacleSensor]);
        getObstacleAbsolutePosition(obstacleAbsolutePosition, pioneerAbsolutePosition, obstacleRelativePosition);

        ROS_INFO("Objeto localizado pelo sensor %d na posicao absoluta (%.2f, %.2f).", closestObstacleSensor, obstacleAbsolutePosition.x, obstacleAbsolutePosition.y);
    }
}

#endif




int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor");
    ros::NodeHandle n;

    pub_cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

#ifdef SONAR
    ros::Subscriber sub = n.subscribe("/sonar", 1, sensorCallBack);
#endif

#ifdef HOKUYO_SCAN
    ros::Subscriber sub = n.subscribe("/hokuyo_scan", 1, sensorCallBack);
#endif

    ros::spin();

    return 0;
}
