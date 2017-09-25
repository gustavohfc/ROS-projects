#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "nav_msgs/Odometry.h"
#include "common.h"

extern Position pioneerPosition;
extern double currentLinearVelocity, currentAngularVelocity;

void poseCallBack(const nav_msgs::Odometry::ConstPtr& msg);

#endif
