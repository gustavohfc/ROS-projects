#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Float32.h"



int main(int argc, char **argv)
{
    ros::init(argc, argv, "direta");
    ros::NodeHandle n;

    ros::Publisher right_pub = n.advertise<std_msgs::Float32>("v_right", 1000);
    ros::Publisher left_pub = n.advertise<std_msgs::Float32>("v_left", 1000);

    // Espera tempo suficiente para finalizar a conexao com os topics
    ros::Duration(1).sleep();

    std_msgs::Float32 v_right, v_left, stop;
    stop.data = 0;

    // Move em um quadrado
    for (int i = 0; i < 4; i++)
    {
        // Anda em linha reta
        v_right.data = v_left.data = 1;
        right_pub.publish(v_right);
        left_pub.publish(v_left);
        ros::Duration(3).sleep();
        right_pub.publish(stop);
        left_pub.publish(stop);

        // Vira 90 graus
        v_right.data = 1;
        v_left.data = -1;
        right_pub.publish(v_right);
        left_pub.publish(v_left);
        ros::Duration(0,523598776).sleep();
        right_pub.publish(stop);
        left_pub.publish(stop);
    }


    // Move em um circulo
    v_right.data = 1;
    v_left.data = 2;
    right_pub.publish(v_right);
    left_pub.publish(v_left);


    // Continua movendo em circulo ate receber um pedido para encerrar
    while (ros::ok());


    // Faz o pioneer parar de se mover
    right_pub.publish(stop);
    left_pub.publish(stop);

    ros::Duration(1).sleep();

    return 0;
}
