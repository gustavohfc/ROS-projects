#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"
#include <cmath>
#include <algorithm>


// Converts degrees to radians.
#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)


// Tolerancia do objetivo
#define XY_GOAL_TOLERANCE 0.2 // metros
#define YAW_GOAL_TOLERANCE 0.2 // metros

#define MIN_LINEAR_VELOCITY 0.2 // m/s
#define MAX_LINEAR_VELOCITY 1 // m/s
#define MIN_ANGULAR_VELOCITY 0.2 // m/s
#define MAX_ANGULAR_VELOCITY 0.7 // m/s

#define MIN_OBJECT_DISTANCE 0.5 // metros
#define DISTANCE_START_BUG_MODE 4 // metros

// Parametros da leitura do sensor laser
#define LASER_ANGLE_MIN -2.35597991943
#define LASER_ANGLE_MAX 2.35597991943
#define LASER_ANGLE_INCREMENT 0.00436696922407
#define LASER_RANGE_ARRAY_SIZE ((LASER_ANGLE_MAX - LASER_ANGLE_MIN) / LASER_ANGLE_INCREMENT)
#define LASER_ARRAY_READING_STEP 4 // +- 1 grau

#define LASER_FRONT_READ_START 0
#define LASER_FRONT_READ_END (degreesToRadians(30))

#define LASER_DIAGONAL_READ_START (degreesToRadians(30))
#define LASER_DIAGONAL_READ_END (degreesToRadians(45))

#define LASER_SIDE_FRONT_READ_START (degreesToRadians(50))
#define LASER_SIDE_FRONT_READ_END (degreesToRadians(70))

#define LASER_SIDE_MIDDLE_READ_START (degreesToRadians(80))
#define LASER_SIDE_MIDDLE_READ_END (degreesToRadians(100))

#define LASER_SIDE_BACK_READ_START (degreesToRadians(110))
#define LASER_SIDE_BACK_READ_END (degreesToRadians(130))



struct Position{ double x, y, yaw; };

// Indica se o Pioneer esta contornando um objeto e qual lado esse objeto esta
enum BugStatus { BugOff, BugLeft, BugRight };
BugStatus pioneerBugStatus = BugOff;

Position pioneerPosition, finalGoal;
double currentLinearVelocity, currentAngularVelocity;

// Menores leituras do sensor laser por direcao
double distanceFrontLeft = 0;
double distanceFrontRight = 0;
double distanceDiagonalLeft = 0;
double distanceDiagonalRight = 0;
double distanceSideFront = 0;
double distanceSideMiddle = 0;
double distanceSideBack = 0;



// Prototipo das funcoes
bool is_linearSpeedZero();
bool is_angularSpeedZero();
bool is_xyReached();
bool is_yawReached();
bool is_goalReached();
void poseCallBack(const nav_msgs::Odometry::ConstPtr& msg);
void hokuyoCallBack(const sensor_msgs::LaserScan& msg);
double getSmallestLaserDistance(double startAngle, double endAngle, const sensor_msgs::LaserScan& msg);
geometry_msgs::Twist calculateTrajectoryVelocity();
void startBugMode();
double calculateTrajectoryLinearVelocity(double goalDistance);
double calculateTrajectoryAngularVelocity(double theta);
double normalizeAngle(double angle);





int main(int argc, char **argv)
{
  ros::init(argc, argv, "Trabalho1");
  ros::NodeHandle n;
  geometry_msgs::Twist vel_msg;

  ros::Publisher pub_cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Subscriber sub_pose = n.subscribe("pose", 1, poseCallBack);
  ros::Subscriber sub = n.subscribe("hokuyo_scan", 1, hokuyoCallBack);

  if (argc != 4)
  {
    ROS_WARN("Numero de parametros invalido, o comando deve ser \'rosrun fcr2017 140021671_Trabalho1 X Y YAW\'");
    return 1;
  }
  finalGoal.x = atof(argv[1]);
  finalGoal.y = atof(argv[2]);
  finalGoal.yaw = normalizeAngle(atof(argv[3]));
  ROS_INFO("Indo para a posicao (%.2f, %.2f, %.2f)", finalGoal.x, finalGoal.y, finalGoal.yaw);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    loop_rate.sleep();

    ros::spinOnce();

    if (is_goalReached())
    {
      ROS_INFO("Objetivo conlcuido, a posicao atual do Pioneer e' (%.2f, %.2f, %.2f)", pioneerPosition.x, pioneerPosition.y, pioneerPosition.yaw);
      break;
    }

    geometry_msgs::Twist vel_msg = calculateTrajectoryVelocity();
    pub_cmd_vel.publish(vel_msg);
  }

  return 0;
}



// Funcoes simples de verificacao
bool is_linearSpeedZero()   { return fabs(currentLinearVelocity) <  0.0001; }
bool is_angularSpeedZero()  { return fabs(currentAngularVelocity) <  0.0001; }
bool is_xyReached()         { return fabs(pioneerPosition.x - finalGoal.x) <  XY_GOAL_TOLERANCE && fabs(pioneerPosition.y - finalGoal.y) <  XY_GOAL_TOLERANCE; }
bool is_yawReached()        { return fabs(pioneerPosition.yaw - finalGoal.yaw) <  YAW_GOAL_TOLERANCE; }
bool is_goalReached()       { return is_linearSpeedZero() && is_angularSpeedZero() && is_xyReached() && is_yawReached(); }
bool is_distanceCritical()  { return distanceFrontLeft < MIN_OBJECT_DISTANCE                                              ||
                                     distanceFrontRight < MIN_OBJECT_DISTANCE                                             ||
                                     (distanceDiagonalLeft * sin(LASER_DIAGONAL_READ_START)) < MIN_OBJECT_DISTANCE / 1.5  ||
                                     (distanceDiagonalRight * sin(LASER_DIAGONAL_READ_START)) < MIN_OBJECT_DISTANCE / 1.5;
                            }



void poseCallBack(const nav_msgs::Odometry::ConstPtr& msg)
{
  pioneerPosition.x = msg->pose.pose.position.x;
  pioneerPosition.y = msg->pose.pose.position.y;

  tf::Pose tfPose;
  tf::poseMsgToTF(msg->pose.pose, tfPose);
  pioneerPosition.yaw = tf::getYaw(tfPose.getRotation());

  currentLinearVelocity = msg->twist.twist.linear.x;
  currentAngularVelocity = msg->twist.twist.angular.z;
}


void hokuyoCallBack(const sensor_msgs::LaserScan& msg)
{
  distanceFrontLeft = getSmallestLaserDistance(LASER_FRONT_READ_START, LASER_FRONT_READ_END, msg);
  distanceFrontRight = getSmallestLaserDistance(-LASER_FRONT_READ_START, -LASER_FRONT_READ_END, msg);
  distanceDiagonalLeft = getSmallestLaserDistance(LASER_DIAGONAL_READ_START, LASER_DIAGONAL_READ_END, msg);
  distanceDiagonalRight = getSmallestLaserDistance(-LASER_DIAGONAL_READ_START, -LASER_DIAGONAL_READ_END, msg);

  if (pioneerBugStatus == BugLeft)
  {
    distanceSideFront = getSmallestLaserDistance(LASER_SIDE_FRONT_READ_START, LASER_SIDE_FRONT_READ_END, msg);
    distanceSideMiddle = getSmallestLaserDistance(LASER_SIDE_MIDDLE_READ_START, LASER_SIDE_MIDDLE_READ_END, msg);
    distanceSideBack = getSmallestLaserDistance(LASER_SIDE_BACK_READ_START, LASER_SIDE_BACK_READ_END, msg);
  }
  else if (pioneerBugStatus == BugRight)
  {
    distanceSideFront = getSmallestLaserDistance(-LASER_SIDE_FRONT_READ_START, -LASER_SIDE_FRONT_READ_END, msg);
    distanceSideMiddle = getSmallestLaserDistance(-LASER_SIDE_MIDDLE_READ_START, -LASER_SIDE_MIDDLE_READ_END, msg);
    distanceSideBack = getSmallestLaserDistance(-LASER_SIDE_BACK_READ_START, -LASER_SIDE_BACK_READ_END, msg);
  }

  // ROS_INFO("distanceFrontLeft: %.2f", distanceFrontLeft);
  // ROS_INFO("distanceFrontRight: %.2f", distanceFrontRight);
  // ROS_INFO("distanceDiagonalLeft: %.2f", distanceDiagonalLeft);
  // ROS_INFO("distanceDiagonalRight: %.2f", distanceDiagonalRight);
  // ROS_INFO("distanceSideFront: %.2f", distanceSideFront);
  // ROS_INFO("distanceSideMiddle: %.2f", distanceSideMiddle);
  // ROS_INFO("distanceSideBack: %.2f", distanceSideBack);
}


// Retorna a menor distancia lida pelo sensor laser dentro de um range de angulo
double getSmallestLaserDistance(double startAngle, double endAngle, const sensor_msgs::LaserScan& msg)
{
  int i, lastIndex;
  double smallestDistance;

  if(startAngle < endAngle)
  {
    i = round((LASER_RANGE_ARRAY_SIZE / 2.0) + (startAngle / LASER_ANGLE_INCREMENT));
    lastIndex = round((LASER_RANGE_ARRAY_SIZE / 2.0) + (endAngle / LASER_ANGLE_INCREMENT));
  }
  else
  {
    lastIndex = round((LASER_RANGE_ARRAY_SIZE / 2.0) + (startAngle / LASER_ANGLE_INCREMENT));
    i = round((LASER_RANGE_ARRAY_SIZE / 2.0) + (endAngle / LASER_ANGLE_INCREMENT));
  }

  smallestDistance = msg.ranges[i];

  for (i += LASER_ARRAY_READING_STEP; i < lastIndex; i += LASER_ARRAY_READING_STEP)
  {
    if (msg.ranges[i] < smallestDistance)
      smallestDistance = msg.ranges[i];
  }

  return smallestDistance;
}


geometry_msgs::Twist calculateTrajectoryVelocity()
{
  geometry_msgs::Twist vel;
  double goalDistanceX, goalDistanceY, goalDistance, goalRelativeTheta;

  // Calcula a posicao relativa do objeto em relacao ao Pioneer
  goalDistanceX = finalGoal.x - pioneerPosition.x;
  goalDistanceY = finalGoal.y - pioneerPosition.y;
  goalDistance = hypot(goalDistanceX, goalDistanceY);
  goalRelativeTheta = normalizeAngle(atan2(goalDistanceY, goalDistanceX) - pioneerPosition.yaw);

  // ROS_INFO("goalDistanceX: %.2f", goalDistanceX);
  // ROS_INFO("goalDistanceY: %.2f", goalDistanceY);
  // ROS_INFO("goalDistance: %.2f", goalDistance);
  // ROS_INFO("goalRelativeTheta: %.2f", goalRelativeTheta * 180 / M_PI);

  if (is_xyReached())
  {
    vel.linear.x = 0;
    if (is_yawReached())
    {
      vel.angular.z = 0; // Chegou no objetivo
    }
    else
    {
      // Corrige o angulo final
      vel.angular.z = calculateTrajectoryAngularVelocity(finalGoal.yaw - pioneerPosition.yaw);
    }
  }
  else
  {
    vel.linear.x = calculateTrajectoryLinearVelocity(goalDistance);

    if (pioneerBugStatus == BugOff)
      startBugMode();

    if (pioneerBugStatus == BugOff)
      vel.angular.z = calculateTrajectoryAngularVelocity(goalRelativeTheta);
    else
      vel.angular.z = calculateTrajectoryAngularVelocity(goalRelativeTheta);
  }

  // Reduz a velocidade linear nas curvas
  vel.linear.x *= 1 - (fabs(vel.angular.z) / MAX_ANGULAR_VELOCITY);

  ROS_INFO("linear: %.3f        angular: %.3f", vel.linear.x, vel.angular.z);

  return vel;
}


void startBugMode()
{
  if (distanceFrontLeft < DISTANCE_START_BUG_MODE || (distanceDiagonalLeft * sin(LASER_DIAGONAL_READ_START)) < DISTANCE_START_BUG_MODE / 1.5)
    pioneerBugStatus = BugLeft;
  else if (distanceFrontRight < DISTANCE_START_BUG_MODE || (distanceDiagonalRight * sin(LASER_DIAGONAL_READ_START)) < DISTANCE_START_BUG_MODE / 1.5)
    pioneerBugStatus = BugRight;
}


double calculateTrajectoryLinearVelocity(double goalDistance)
{
  if (is_distanceCritical())
    return 0;

  double linearVelocity =  MAX_LINEAR_VELOCITY;

  // double pointDistance = std::min(goalDistance,
  //                                 std::min(std::min(distanceFrontLeft, distanceFrontRight) - MIN_OBJECT_DISTANCE,
  //                                          std::min(distanceDiagonalLeft, distanceDiagonalRight) - (MIN_OBJECT_DISTANCE / 2.5)));
  //
  // if (pointDistance <= 0)
  //   return 0;

  if (goalDistance < 2.5)
    linearVelocity -= MAX_LINEAR_VELOCITY * (1 - goalDistance / 2.5);

  if (linearVelocity > MAX_LINEAR_VELOCITY)
  {
    ROS_INFO("Linear velocity too high");
    return MAX_LINEAR_VELOCITY;
  }
  if (linearVelocity < MIN_LINEAR_VELOCITY)
  {
    return MIN_LINEAR_VELOCITY;
  }
  else
  {
    return linearVelocity;
  }
}


double calculateTrajectoryAngularVelocity(double theta)
{
  theta = normalizeAngle(theta);
  if (fabs(theta) < YAW_GOAL_TOLERANCE / 2)
    return 0;

  double angularVelocity = MAX_ANGULAR_VELOCITY;
  int direcao = (theta > 0) ? 1 : -1;

  if (fabs(theta) < M_PI_2)
    angularVelocity -= MAX_ANGULAR_VELOCITY * (1 - fabs(theta) / M_PI_2);

  if (angularVelocity > MAX_ANGULAR_VELOCITY)
  {
    ROS_INFO("Angular velocity too high");
    return direcao * MAX_ANGULAR_VELOCITY;
  }
  if (angularVelocity < MIN_ANGULAR_VELOCITY)
  {
    return direcao * MIN_ANGULAR_VELOCITY;
  }
  else
  {
    return direcao * angularVelocity;
  }
}


// Retorna o angulo equivalente entre PI e -PI
double normalizeAngle(double angle)
{
  angle = fmod(angle + M_PI, 2 * M_PI);
  if (angle < 0)
    angle += 2 * M_PI;

  return angle - M_PI;
}
