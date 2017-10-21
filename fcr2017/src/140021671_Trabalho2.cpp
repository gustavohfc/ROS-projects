#include <ros/ros.h>
#include <ros/console.h>

#include "common_lib/odometer.h"
#include "common_lib/laser_sensor.h"
#include "common_lib/motion_controller.h"
#include "common_lib/graph.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Trabalho2");
    ros::NodeHandle nodeHandle;

    // Set verbosity level to debug
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    if (argc != 2)
    {
        ROS_ERROR("Numero de parametros invalido, deve ser passado apenas um parametro correspondente ao caminho para o arquivo de informacoes do grafo");
        exit(EXIT_FAILURE);
    }


    // Initialize objects
    PioneerState current_state(GoingToXY);
    Odometer odometer(nodeHandle);
    LaserSensor laser_sensor(nodeHandle);
    MotionController motion_controller(nodeHandle, current_state, odometer, laser_sensor);
    Graph graph(argv[1]);

    return 0;

    // temp
    motion_controller.addGoal(Position(-27, 4.5));

    ros::Rate loop_rate(60);

    while (ros::ok())
    {
        loop_rate.sleep();

        ros::spinOnce();

        // Wait until receive at least one message from the laser sensor
        if (!laser_sensor.msg)
            continue;

        motion_controller.goToGoal();

        if (!motion_controller.hasGoals())
            break;

        ROS_DEBUG("\n\n\n\n");
    }

    loop_rate.sleep();

    return 0;
}
