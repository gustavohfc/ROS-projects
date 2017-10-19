#include <ros/ros.h>

#include "common.h"

// Retorna o angulo equivalente entre PI e -PI
double normalizeAngle(double angle)
{
    angle = fmod(angle + M_PI, 2 * M_PI);
    if (angle < 0)
        angle += 2 * M_PI;

    return angle - M_PI;
}