#include <ros/ros.h>
#include <ros/console.h>

#include "laser_line_extraction/LineSegmentList.h"

#include "common.h"
#include "odometer.h"
#include "laser_sensor.h"
#include "user_motion_controller.h"
#include "graph.h"


#define LOOP_RATE 120

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Trabalho3");
    ros::NodeHandle nodeHandle;

    // Set verbosity level to debug
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    // Initialize objects
    LaserSensor laser_sensor(nodeHandle);
    UserMotionController motion_controller(nodeHandle, laser_sensor);

    // Wait until receive at least one message from each sensor
    while (!laser_sensor.msg)
    {
        ros::spinOnce();
    }

    ros::Rate loop_rate(LOOP_RATE);

    while (ros::ok())
    {
        loop_rate.sleep();

        ros::spinOnce();

        motion_controller.move();
    }

    return 0;
}
