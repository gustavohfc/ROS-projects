#include <ros/ros.h>
#include <ros/console.h>

#include "common_lib/odometer.h"
#include "common_lib/laser_sensor.h"
#include "common_lib/motion_controller.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Trabalho2");
    ros::NodeHandle nodeHandle;

    // Set verbosity level to debug
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }


    // Initialize objects
    PioneerState current_state(GoingToXY);
    Odometer odometer(nodeHandle);
    LaserSensor laser_sensor(nodeHandle);
    MotionController motion_controller(nodeHandle, current_state, odometer);



    // temp
    motion_controller.addGoal(Position(1, 0));
    motion_controller.addGoal(Position(1, 1));
    motion_controller.addGoal(Position(0, 1));
    motion_controller.addGoal(Position(0, 0));

    ros::Rate loop_rate(60);

    while (ros::ok())
    {
        loop_rate.sleep();

        ros::spinOnce();

        // Espera ate ter pelo menos uma leitura do sensor laser
        // if (!laserSensor.msg)
        //     continue;

        // gridMap.update();

        motion_controller.goToGoal();

        if (!motion_controller.hasGoals())
            break;
    }

    // gridMap.save();
    loop_rate.sleep();

    return 0;
}
