#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <nav_msgs/Odometry.h>
#include "common.h"


class Odometer
{
private:
    nav_msgs::Odometry::ConstPtr msg;
    double x, y, yaw, linearVelocity, angularVelocity;

public:
    void init(ros::NodeHandle& nodeHandle, ros::Subscriber subscriber);
    void callback(const nav_msgs::Odometry::ConstPtr& msg);

    double getX()               const { return x; };
    double getY()               const { return y; };
    double getYaw()             const { return yaw; };
    double getLinearVelocity()  const { return linearVelocity; };
    double getAngularVelocity() const { return angularVelocity; };
};


// Global extern variable
extern Odometer odometer;


#endif
