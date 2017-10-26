#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <nav_msgs/Odometry.h>
#include "common.h"

#define LASER_SENSOR_DISTANCE_FROM_CENTER 0.2


class Odometer
{
private:
    ros::Subscriber sub_odom;
    double x, y; // Pioneer absolute position
    double yaw; // Pioneer absolute orientation
    double laserSensorX, laserSensorY; // Pioneer's laser sensor absolute position
    double linearVelocity, angularVelocity; // Pioneer absolute velocity

public:
    nav_msgs::Odometry::ConstPtr msg;

    Odometer(ros::NodeHandle& nodeHandle);

    void callback(const nav_msgs::Odometry::ConstPtr& msg);

    double getX()               const { return x; };
    double getY()               const { return y; };
    double getYaw()             const { return yaw; };
    double getLaserSensorX()    const { return laserSensorX; };
    double getLaserSensorY()    const { return laserSensorY; };
    double getLinearVelocity()  const { return linearVelocity; };
    double getAngularVelocity() const { return angularVelocity; };
};


#endif
