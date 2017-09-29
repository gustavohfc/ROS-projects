#include <ros/ros.h>
#include <ros/console.h>

#include "common_lib/laser_sensor.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "roteiro5");
    ros::NodeHandle nodeHandle;
    ros::Subscriber laserSubriber;

    // Set debug verbosity level
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }


    laserSensor.init(nodeHandle, laserSubriber);

    // ros::Subscriber sub = n.subscribe("hokuyo_scan", 1, &LaserSensor::callback, &laserSensor);

    // laserSensor.sub.shutdown();
    
    return 0;
}
