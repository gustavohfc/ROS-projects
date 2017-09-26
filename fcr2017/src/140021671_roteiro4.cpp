#include <ros/ros.h>
#include <ros/console.h>
#include <deque>

#include "common_lib/common.h"
#include "common_lib/odometry.h"
#include "common_lib/laser_sensor.h"
#include "common_lib/graph.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "roteiro4");
    ros::NodeHandle n;

    // Set debug verbosity level
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    std::deque<Position> goals;

    Graph cicGraph;
    cicGraph.findPathDijkstra(goals);

    ros::Publisher pub_cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Subscriber sub_pose = n.subscribe("pose", 1, poseCallBack);

    pioneerState = GoingToPoint;

    ros::Rate loop_rate(60);

    while (ros::ok())
    {
        loop_rate.sleep();

        ros::spinOnce();

        updateGoals(goals);

        if (goals.empty())
        {
            ROS_INFO("Completou todos os objetivos."); 
            break;
        }

        goToPosition(goals.front(), pub_cmd_vel);
    }

    return 0;
}
