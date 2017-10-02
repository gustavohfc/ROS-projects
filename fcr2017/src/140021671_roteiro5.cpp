#include <ros/ros.h>
#include <ros/console.h>

#include "common_lib/common.h"
#include "common_lib/laser_sensor.h"
#include "common_lib/odometer.h"
#include "common_lib/grid_map.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "roteiro5");
    ros::NodeHandle nodeHandle;
    ros::Subscriber laserSubriber, odometerSubscriber;

    // Set debug verbosity level
    // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    //     ros::console::notifyLoggerLevelsChanged();
    // }

    ros::Publisher pub_cmd_vel = nodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    laserSensor.init(nodeHandle, laserSubriber);
    odometer.init(nodeHandle, odometerSubscriber);
    gridMap.init(nodeHandle);

    Position goal(39, 0);

    ros::Rate loop_rate(60);

    while (ros::ok())
    {
        loop_rate.sleep();

        ros::spinOnce();

        // Espera ate ter pelo menos uma leitura do sensor laser
        if (!laserSensor.msg)
            continue;

        gridMap.update();

        if (is_goalReached(goal))
            break;

        goToPosition(goal, pub_cmd_vel);
    }

    gridMap.save();

    return 0;
}
