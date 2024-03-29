#include <ros/ros.h>
#include <ros/console.h>

#include "common.h"
#include "odometer.h"
#include "laser_sensor.h"
#include "motion_controller.h"
#include "graph.h"


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

    if (motion_controller.hasGoals())
        current_state.state = GoingToXY;
    else
    {
        ROS_INFO("Ja esta no no inicial");
        current_state.state == WaitingGoal;
        return;
    }

    ROS_INFO("Indo para o no inicial [ Atual(%c) -> %s ]", graph.getCurrentNode()->ID, motion_controller.getGoalsString().c_str());

    ros::Rate loop_rate(LOOP_RATE);

    while (ros::ok())
    {
        loop_rate.sleep();

        ros::spinOnce();

        motion_controller.goToGoal();

        if (current_state.state == GoalReached)
        {
            ROS_INFO("Chegou na posicao inicial [%c].", graph.getCurrentNode()->ID);
            current_state.state == WaitingGoal;
            break;
        }
    }
}



void fillGridMap(Graph& graph, MotionController& motion_controller, PioneerState& current_state)
{
    motion_controller.addGoals(graph.closeLoopPathToGoals());

    current_state.state = GoingToXY;

    ROS_INFO("Construindo as grades de ocupacao [ Atual(%c) -> %s ]", graph.getCurrentNode()->ID, motion_controller.getGoalsString().c_str());

    ros::Rate loop_rate(LOOP_RATE);

    graph.updateAllMapsOnRviz();

    while (ros::ok())
    {
        loop_rate.sleep();

        ros::spinOnce();

        motion_controller.goToGoal();

        if (graph.getCurrentNode() != NULL)
        {
            graph.getCurrentNode()->map->updateGrid();
            graph.getCurrentNode()->map->sendToRviz();
        }

        if (current_state.state == GoalReached)
        {
            ROS_INFO("Grades de ocupacao completas");
            current_state.state == WaitingGoal;
            break;
        }
    }

    graph.saveImageFiles();
}



void goToFinalPosition(Graph& graph, MotionController& motion_controller, PioneerState& current_state)
{
    char final_node_ID;

    // Get the final node
    std::cout << "\nDigite o nó final <A - R>: ";
    std::cin >> final_node_ID;
    while (std::cin.fail() || graph.getNode(final_node_ID, false) == NULL)
    {
        std::cout << "Nó final inválido, digite novamente <A - R>: ";
        std::cin >> final_node_ID;
    }

    motion_controller.addGoals(graph.Dijkstra(final_node_ID));

    if (motion_controller.hasGoals())
        current_state.state = GoingToXY;
    else
    {
        ROS_INFO("Ja esta no no final");
        current_state.state == WaitingGoal;
        return;
    }

    ROS_INFO("Indo para o no final [ Atual(%c) -> %s ]", graph.getCurrentNode()->ID, motion_controller.getGoalsString().c_str());

    ros::Rate loop_rate(LOOP_RATE);

    while (ros::ok())
    {
        loop_rate.sleep();

        ros::spinOnce();

        motion_controller.goToGoal();

        if (current_state.state == GoalReached)
        {
            ROS_INFO("Chegou na posicao final [%c].", graph.getCurrentNode()->ID);
            current_state.state == WaitingGoal;
            break;
        }
    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "Trabalho2");
    ros::NodeHandle nodeHandle;

    // Set verbosity level to debug
    // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    //     ros::console::notifyLoggerLevelsChanged();
    // }


    // Wait for all messages from other nodes be displayed in the terminal
    ros::Duration(1).sleep();


    if (argc != 2)
    {
        ROS_ERROR("Numero de parametros invalido, deve ser passado apenas um parametro correspondente ao caminho a pasta fcr2017");
        exit(EXIT_FAILURE);
    }

    // Initialize objects
    PioneerState current_state(WaitingGoal);
    Odometer odometer(nodeHandle);
    LaserSensor laser_sensor(nodeHandle);
    Graph graph(argv[1], nodeHandle, odometer, laser_sensor);
    MotionController motion_controller(nodeHandle, current_state, odometer, laser_sensor, graph);

    // Wait until receive at least one message from each sensor
    while (!laser_sensor.msg || !odometer.msg)
        ros::spinOnce();


    goToInitialPosition(graph, motion_controller, current_state);
    fillGridMap(graph, motion_controller, current_state);
    goToFinalPosition(graph, motion_controller, current_state);

    // Wait to make sure all messages were send
    ros::Duration(1).sleep();

    return 0;
}
