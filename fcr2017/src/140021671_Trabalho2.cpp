#include <ros/ros.h>
#include <ros/console.h>

#include "common_lib/common.h"
#include "common_lib/odometer.h"
#include "common_lib/laser_sensor.h"
#include "common_lib/motion_controller.h"
#include "common_lib/graph.h"


#define LOOP_RATE 60


void goToInitialPosition(Graph& graph, MotionController& motion_controller, PioneerState& current_state)
{
    char initial_node_ID;

    // Get the initial node
    std::cout << "\n\n\n\n\n\n\nDigite o nó inicial <A - R>: ";
    std::cin >> initial_node_ID;
    while (std::cin.fail() || graph.getNode(initial_node_ID, false) == NULL)
    {
        std::cout << "Nó inicial inválido, digite novamente <A - R>: ";
        std::cin >> initial_node_ID;
    }

    motion_controller.addGoals(graph.Dijkstra(initial_node_ID));
    current_state.state = GoingToXY;

    ROS_INFO("Indo para o no inicial [ Atual(%c) -> %s ]", graph.getCurrentNode()->ID, motion_controller.getGoalsString().c_str());

    ros::Rate loop_rate(LOOP_RATE);

    while (ros::ok())
    {
        loop_rate.sleep();

        ros::spinOnce();

        motion_controller.goToGoal();

        if (current_state.state == GoalReached)
        {
            ROS_INFO("Chegou na posicao inicial.");
            break;
        }
    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "Trabalho2");
    ros::NodeHandle nodeHandle;

    // Set verbosity level to debug
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }


    // Wait for all messages from other nodes be displayed in the terminal
    ros::Duration(1).sleep();


    if (argc != 2)
    {
        ROS_ERROR("Numero de parametros invalido, deve ser passado apenas um parametro correspondente ao caminho para o arquivo de informacoes do grafo");
        exit(EXIT_FAILURE);
    }

    // Initialize objects
    PioneerState current_state(WaitingGoal);
    Odometer odometer(nodeHandle);
    LaserSensor laser_sensor(nodeHandle);
    MotionController motion_controller(nodeHandle, current_state, odometer, laser_sensor);
    Graph graph(argv[1], odometer);

    // Wait until receive at least one message from each sensor
    while (!laser_sensor.msg || !odometer.msg)
        ros::spinOnce();

    goToInitialPosition(graph, motion_controller, current_state);

    // temp
    // motion_controller.addGoal(Position(10, 0));

    // ros::Rate loop_rate(60);

    // while (ros::ok())
    // {
    //     loop_rate.sleep();

    //     ros::spinOnce();

    //     // Wait until receive at least one message from the laser sensor
    //     if (!laser_sensor.msg)
    //         continue;

    //     motion_controller.goToGoal();

    //     if (!motion_controller.hasGoals())
    //         break;

    //     ROS_DEBUG("\n\n\n\n");
    // }

    // loop_rate.sleep();

    // Wait to make sure all messages were send
    ros::Duration(1).sleep();

    return 0;
}
