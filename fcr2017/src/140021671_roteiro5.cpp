#include <ros/ros.h>
#include <ros/console.h>

#include "common_lib/laser_sensor.h"
#include "common_lib/odometer.h"
#include "common_lib/grid_map.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "roteiro5");
    ros::NodeHandle nodeHandle;
    ros::Subscriber laserSubriber, odometerSubscriber;

    // Set debug verbosity level
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }


    laserSensor.init(nodeHandle, laserSubriber);
    odometer.init(nodeHandle, odometerSubscriber);
    gridMap.init(nodeHandle);

    ros::Rate loop_rate(60);

    // ros::Time begin, end;
    // ros::Duration d;

    while (ros::ok())
    {
        // begin = ros::Time::now();
        loop_rate.sleep();
        // end = ros::Time::now();
        // d = end - begin;
        // ROS_INFO("%d", d.nsec);


        ros::spinOnce();

        // Espera ate ter pelo menos uma leitura do sensor laser
        if (!laserSensor.msg)
            continue;

        gridMap.update();

        break;
    }

    gridMap.save();
    
    return 0;
}
