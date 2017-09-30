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

    int loop_count = 0; // TEMP
    while (ros::ok() )
    {
        loop_rate.sleep();

        // ros::spinOnce();

        gridMap.update();
    }

    // gridMap.save();
    
    return 0;
}
