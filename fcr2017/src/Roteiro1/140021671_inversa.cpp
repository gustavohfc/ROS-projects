#include "ros/ros.h"
#include "ros/time.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "inversa");
  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  // Espera tempo suficiente para finalizar a conexao com o topic
  ros::Duration(1).sleep();

  geometry_msgs::Twist vel;
  geometry_msgs::Twist stop;
  stop.linear.x = 0;
  stop.angular.z = 0;

  // Move em um quadrado
  for (int i = 0; i < 4; i++)
  {
    // Anda em linha reta
    vel.linear.x = 1;
    vel.angular.z = 0;
    pub.publish(vel);
    ros::Duration(3).sleep();
    pub.publish(stop);

    // Vira 90 graus
    vel.linear.x = 0;
    vel.angular.z = 1;
    pub.publish(vel);
    ros::Duration(1.5707963268).sleep();
    pub.publish(stop);
  }


  // Move em um circulo
  vel.linear.x = 1;
  vel.angular.z = 1;
  pub.publish(vel);


  // Continua movendo em circulo ate receber um pedido para encerrar
  while (ros::ok());


  // Faz o pioneer parar de se mover
  pub.publish(stop);

  ros::Duration(1).sleep();

  return 0;
}
