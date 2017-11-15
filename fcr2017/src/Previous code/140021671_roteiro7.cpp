#include <ros/ros.h>
#include <ros/console.h>

#include "laser_line_extraction/LineSegmentList.h"

#include "common.h"
#include "odometer.h"
#include "laser_sensor.h"
#include "motion_controller.h"
#include "graph.h"


#define LOOP_RATE 60

int main(int argc, char **argv)
{
    ros::init(argc, argv, "roteiro7");
    ros::NodeHandle nodeHandle;

    // Set verbosity level to debug
    // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    //     ros::console::notifyLoggerLevelsChanged();
    // }


    // Initialize objects
    PioneerState current_state(WaitingGoal);
    Odometer odometer(nodeHandle);
    LaserSensor laser_sensor(nodeHandle);
    Graph graph(argv[1], nodeHandle, odometer, laser_sensor);
    MotionController motion_controller(nodeHandle, current_state, odometer, laser_sensor, graph);

    return 0;
}
