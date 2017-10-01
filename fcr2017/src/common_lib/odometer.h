#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <nav_msgs/Odometry.h>
#include "common.h"

#define LASER_SENSOR_DISTANCE_FROM_CENTER 0.2

class Odometer
{
private:
    nav_msgs::Odometry::ConstPtr msg;
    double x, y, yaw;
    double laserSensorX, laserSensorY;
    double linearVelocity, angularVelocity;

public:
    void init(ros::NodeHandle& nodeHandle, ros::Subscriber& subscriber);
    void callback(const nav_msgs::Odometry::ConstPtr& msg);

    double getX()               const { return x; };
    double getY()               const { return y; };
    double getYaw()             const { return yaw; };
    double getLaserSensorX()    const { return laserSensorX; };
    double getLaserSensorY()    const { return laserSensorY; };
    double getLinearVelocity()  const { return linearVelocity; };
    double getAngularVelocity() const { return angularVelocity; };
};


// Global extern variable
extern Odometer odometer;


#endif
