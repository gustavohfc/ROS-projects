#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"
#include <cmath>
#include <algorithm>


// Converts degrees to radians.
#define degreesToRadians(angleDegrees) ((angleDegrees) * M_PI / 180.0)


// Tolerancia do objetivo
#define XY_GOAL_TOLERANCE 0.2 // metros
#define YAW_GOAL_TOLERANCE 0.05 // metros

#define YAW_TOLERANCE 0.02

#define MIN_LINEAR_VELOCITY 0.2 // m/s
#define MAX_LINEAR_VELOCITY 1 // m/s
#define MIN_ANGULAR_VELOCITY 0.2 // m/s
#define MAX_ANGULAR_VELOCITY 0.7 // m/s

#define MIN_OBJECT_DISTANCE 0.5 // metros
#define DISTANCE_START_BUG2_MODE 1 // metros
#define BUG2_WALL_DISTANCE 1
#define BUG2_WALL_TOLERANCE 0.20
#define BUG2_RELEVANT_DISTANCE 3
#define BUG_LINE_DISTANCE_TOLERANCE 1

// Parametros da leitura do sensor laser
#define LASER_RANGE_ARRAY_SIZE ((LASER_ANGLE_MAX - LASER_ANGLE_MIN) / LASER_ANGLE_INCREMENT)

#define LASER_ARRAY_FRONT_READING_STEP 4 // +- 1 grau
#define LASER_ARRAY_SIDE_READING_STEP 1

#define LASER_FRONT_READ_START 0
#define LASER_FRONT_READ_END (degreesToRadians(20))

#define LASER_DIAGONAL_READ_START (degreesToRadians(20))
#define LASER_DIAGONAL_READ_END (degreesToRadians(45))

#define LASER_SIDE_FRONT_READ_START (degreesToRadians(59.5))
#define LASER_SIDE_FRONT_READ_END (degreesToRadians(60.5))
#define LASER_SIDE_FRONT_ANGLE (degreesToRadians(90 - 60))

#define LASER_SIDE_MIDDLE_READ_START (degreesToRadians(89.5))
#define LASER_SIDE_MIDDLE_READ_END (degreesToRadians(90.5))

#define LASER_SIDE_BACK_READ_START (degreesToRadians(119.5))
#define LASER_SIDE_BACK_READ_END (degreesToRadians(120.5))
#define LASER_SIDE_BACK_ANGLE (degreesToRadians(120 - 90))



struct Position{ double x, y, yaw; };

// Indica se o Pioneer esta contornando um objeto e qual lado esse objeto esta
enum Status { WaitingForGoal, GoingToGoal, Bug2ModeLeft, Bug2ModeRight };
Status pioneerStatus = WaitingForGoal;

Position pioneerPosition, finalGoal;
double currentLinearVelocity, currentAngularVelocity;
double bugLineStartX, bugLineStartY;

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
double getSmallestLaserDistance(const double angle1, const double angle2, const double step, const sensor_msgs::LaserScan& msg);
void navGoalCallBack(const geometry_msgs::PoseStamped& msg);
geometry_msgs::Twist calculateTrajectoryVelocity();
void updatePioneerStatus(double goalRelativeTheta);
double calculateTrajectoryLinearVelocity(const double goalDistance);
double velocityToAngle(double theta);
geometry_msgs::Twist calculateBug2Velocity();
double normalizeAngle(double angle);





int main(int argc, char **argv)
{
    ros::init(argc, argv, "Trabalho1");
    ros::NodeHandle n;
    geometry_msgs::Twist vel_msg;

    ros::Publisher pub_cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Subscriber sub_pose = n.subscribe("pose", 1, poseCallBack);
    ros::Subscriber sub_hokuyo_scan = n.subscribe("hokuyo_scan", 1, hokuyoCallBack);
    ros::Subscriber sub_goal = n.subscribe("move_base_simple/goal", 1, navGoalCallBack);


    // ROS_INFO("Indo para a posicao (%.2f, %.2f, %.2f)", finalGoal.x, finalGoal.y, finalGoal.yaw);

    ros::Rate loop_rate(60);

    while (ros::ok())
    {
        loop_rate.sleep();

        ros::spinOnce();

        if (pioneerStatus == WaitingForGoal)
            continue;

        if (is_goalReached() && pioneerStatus != WaitingForGoal)
        {
            ROS_INFO("Objetivo conlcuido, a posicao atual do Pioneer e' (%.2f, %.2f, %.2f)", pioneerPosition.x, pioneerPosition.y, pioneerPosition.yaw);
            pioneerStatus = WaitingForGoal;
        }

        vel_msg = calculateTrajectoryVelocity();
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
bool is_distanceCritical()  { return    distanceFrontLeft < MIN_OBJECT_DISTANCE     ||
                                        distanceFrontRight < MIN_OBJECT_DISTANCE    ||
                                        distanceDiagonalLeft < MIN_OBJECT_DISTANCE  ||
                                        distanceDiagonalRight < MIN_OBJECT_DISTANCE;
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
    distanceFrontLeft = getSmallestLaserDistance(LASER_FRONT_READ_START, LASER_FRONT_READ_END, LASER_ARRAY_FRONT_READING_STEP, msg);
    distanceFrontRight = getSmallestLaserDistance(-LASER_FRONT_READ_START, -LASER_FRONT_READ_END, LASER_ARRAY_FRONT_READING_STEP, msg);
    distanceDiagonalLeft = getSmallestLaserDistance(LASER_DIAGONAL_READ_START, LASER_DIAGONAL_READ_END, LASER_ARRAY_FRONT_READING_STEP, msg);
    distanceDiagonalRight = getSmallestLaserDistance(-LASER_DIAGONAL_READ_START, -LASER_DIAGONAL_READ_END, LASER_ARRAY_FRONT_READING_STEP, msg);


    if (pioneerStatus == Bug2ModeLeft)
    {
        distanceSideFront = getSmallestLaserDistance(LASER_SIDE_FRONT_READ_START, LASER_SIDE_FRONT_READ_END, LASER_ARRAY_SIDE_READING_STEP, msg);
        distanceSideMiddle = getSmallestLaserDistance(LASER_SIDE_MIDDLE_READ_START, LASER_SIDE_MIDDLE_READ_END, LASER_ARRAY_SIDE_READING_STEP, msg);
        distanceSideBack = getSmallestLaserDistance(LASER_SIDE_BACK_READ_START, LASER_SIDE_BACK_READ_END, LASER_ARRAY_SIDE_READING_STEP, msg);
    }
    else if (pioneerStatus == Bug2ModeRight)
    {
        distanceSideFront = getSmallestLaserDistance(-LASER_SIDE_FRONT_READ_START, -LASER_SIDE_FRONT_READ_END, LASER_ARRAY_SIDE_READING_STEP, msg);
        distanceSideMiddle = getSmallestLaserDistance(-LASER_SIDE_MIDDLE_READ_START, -LASER_SIDE_MIDDLE_READ_END, LASER_ARRAY_SIDE_READING_STEP, msg);
        distanceSideBack = getSmallestLaserDistance(-LASER_SIDE_BACK_READ_START, -LASER_SIDE_BACK_READ_END, LASER_ARRAY_SIDE_READING_STEP, msg);
    }

    // ROS_INFO("distanceFrontLeft: %f", distanceFrontLeft);
    // ROS_INFO("distanceFrontRight: %f", distanceFrontRight);
    // ROS_INFO("distanceDiagonalLeft: %f", distanceDiagonalLeft);
    // ROS_INFO("distanceDiagonalRight: %f", distanceDiagonalRight);
    // ROS_INFO("distanceSideFront: %f", distanceSideFront);
    // ROS_INFO("distanceSideMiddle: %f", distanceSideMiddle);
    // ROS_INFO("distanceSideBack: %f", distanceSideBack);
}


// Retorna a menor distancia lida pelo sensor laser dentro de um range de angulo
double getSmallestLaserDistance(const double angle1, const double angle2, const double step, const sensor_msgs::LaserScan& msg)
{
    int i, lastIndex, index1, index2;
    double smallestDistance;

    static const double angleIncrement = msg.angle_increment;
    static const double angleMin = msg.angle_min;
    static const double angleMax = msg.angle_max;
    static const double vectorSize = msg.ranges.size();

    index1 = (vectorSize / 2) + (angle1 / angleIncrement);
    index2 = (vectorSize / 2) + (angle2 / angleIncrement);

    if (index1 < index2)
    {
        i = index1;
        lastIndex = index2;
    }
    else
    {
        i = index2;
        lastIndex = index1;
    }

    smallestDistance = msg.ranges[i];

    for (i += step; i < lastIndex; i += step)
    {
        if (msg.ranges[i] < smallestDistance)
            smallestDistance = msg.ranges[i];
    }

    return smallestDistance;
}


void navGoalCallBack(const geometry_msgs::PoseStamped& msg)
{
    finalGoal.x = msg.pose.position.x;
    finalGoal.y = msg.pose.position.y;

    tf::Pose tfPose;
    tf::poseMsgToTF(msg.pose, tfPose);
    finalGoal.yaw = tf::getYaw(tfPose.getRotation());

    ROS_INFO("Indo para a posicao (%.2f, %.2f, %.2f)", finalGoal.x, finalGoal.y, finalGoal.yaw);
    pioneerStatus = GoingToGoal;
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
            vel.angular.z = velocityToAngle(finalGoal.yaw - pioneerPosition.yaw);
        }
    }
    else
    {
        updatePioneerStatus(goalRelativeTheta);

        if (pioneerStatus == GoingToGoal)
        {
            vel.linear.x = calculateTrajectoryLinearVelocity(goalDistance);
            vel.angular.z = velocityToAngle(goalRelativeTheta);

            // Reduz a velocidade linear nas curvas
            vel.linear.x *= 1 - (fabs(vel.angular.z) / MAX_ANGULAR_VELOCITY);
        }
        else if (pioneerStatus == Bug2ModeLeft || pioneerStatus == Bug2ModeRight)
        {
            vel = calculateBug2Velocity();
        }
    }

    // ROS_INFO("linear: %f        angular: %f", vel.linear.x, vel.angular.z);

    return vel;
}


void updatePioneerStatus(double goalRelativeTheta)
{
    if (pioneerStatus == GoingToGoal)
    {
        if (distanceFrontRight < DISTANCE_START_BUG2_MODE)
        {
            pioneerStatus = Bug2ModeRight;
            bugLineStartX = pioneerPosition.x;
            bugLineStartY = pioneerPosition.y;
        }
        else if (distanceFrontLeft < DISTANCE_START_BUG2_MODE)
        {
            pioneerStatus = Bug2ModeLeft;
            bugLineStartX = pioneerPosition.x;
            bugLineStartY = pioneerPosition.y;
        }
    }
    else if (pioneerStatus == Bug2ModeLeft || pioneerStatus == Bug2ModeRight)
    {
        double lineDistance = fabs((finalGoal.x - bugLineStartX) * (bugLineStartY - pioneerPosition.y) - (bugLineStartX - pioneerPosition.x) * (finalGoal.y - bugLineStartY)) /
                              sqrt(pow(finalGoal.y  - bugLineStartY, 2) + pow(finalGoal.x - bugLineStartX, 2));

        double startBugDistance = sqrt(pow(pioneerPosition.y  - bugLineStartY, 2) + pow(pioneerPosition.x - bugLineStartX, 2));

        bool wallOnWay = (pioneerStatus == Bug2ModeLeft ? goalRelativeTheta > 0 : goalRelativeTheta < 0);

        // ROS_INFO("lineDistance = %f\tstartBugDistance: %f", lineDistance, startBugDistance);
        // ROS_INFO("bugStart = %f  %f", bugLineStartX, bugLineStartY);
        // ROS_INFO("wallOnWay = %d", wallOnWay);

        if (lineDistance < BUG_LINE_DISTANCE_TOLERANCE && startBugDistance > 1.5 * BUG_LINE_DISTANCE_TOLERANCE && !wallOnWay)
            pioneerStatus = GoingToGoal;
    }
}


double calculateTrajectoryLinearVelocity(const double goalDistance)
{
    if (is_distanceCritical())
        return 0;

    double linearVelocity =  MAX_LINEAR_VELOCITY;

    // Reduz a velocidade quando está chegando no objetivo
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


double velocityToAngle(double theta)
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


geometry_msgs::Twist calculateBug2Velocity()
{
    geometry_msgs::Twist vel;

    static bool goingToWallDistance = true;

    double lateralDistanceFront = cos(LASER_SIDE_FRONT_ANGLE) * distanceSideFront;
    double lateralDistanceMiddle = distanceSideMiddle;
    double lateralDistanceBack = cos(LASER_SIDE_BACK_ANGLE) * distanceSideBack;

    bool isFrontRelevant = lateralDistanceFront < BUG2_RELEVANT_DISTANCE;
    bool isMiddleRelevant = lateralDistanceMiddle < BUG2_RELEVANT_DISTANCE;
    bool isBackRelevant = lateralDistanceBack < BUG2_RELEVANT_DISTANCE;

    // ROS_INFO("%d %d %d", isFrontRelevant, isMiddleRelevant, isBackRelevant);



    if (!isFrontRelevant && !isMiddleRelevant && !isBackRelevant)
    {
        // Rotaciona ate algum sensor lateral ter alguma informacao
        vel.linear.x = 0;
        vel.angular.z = MIN_ANGULAR_VELOCITY;
    }
    else if (isFrontRelevant && !isMiddleRelevant && !isBackRelevant)
    {
        if (distanceFrontLeft > MIN_OBJECT_DISTANCE && distanceFrontRight > MIN_OBJECT_DISTANCE)
            vel.linear.x = 0.5;
        else
            vel.linear.x = 0;

        vel.angular.z = -0.5;
    }
    else if (!isFrontRelevant && !isMiddleRelevant && isBackRelevant)
    {
        // Faz uma curva no canto do obstaculo
        vel.linear.x = 0.5;
        vel.angular.z = 0.5;
    }
    else if (isFrontRelevant && !isMiddleRelevant && isBackRelevant)
    {
        if (std::min(distanceSideFront, distanceSideBack) > MIN_OBJECT_DISTANCE)
        {
            vel.linear.x = MIN_LINEAR_VELOCITY;
            vel.angular.z = 0;
        }
        else
        {
            vel.linear.x = 0;
            vel.angular.z = MIN_ANGULAR_VELOCITY;
        }
    }
    else if (!isFrontRelevant && isMiddleRelevant && !isBackRelevant)
    {
        if (distanceSideMiddle > MIN_OBJECT_DISTANCE)
        {
            vel.linear.x = MIN_LINEAR_VELOCITY;
            vel.angular.z = MIN_ANGULAR_VELOCITY;
        }
        else
        {
            vel.linear.x = 0;
            vel.angular.z = MIN_ANGULAR_VELOCITY;
        }
    }
    else if (isMiddleRelevant)
    {
        double lateralAngle, lateralAngleFront, lateralAngleBack;

        double menorDistancia = std::min(std::min(distanceSideBack, distanceSideMiddle), distanceSideFront);

        // Calcula o angulo entre o objeto lateral e a trajetoria do pioneer
        if (isFrontRelevant)
            lateralAngleFront = -1 * atan((distanceSideMiddle - lateralDistanceFront) / (sin(LASER_SIDE_FRONT_ANGLE) * distanceSideFront));
        if (isBackRelevant)
            lateralAngleBack = atan((distanceSideMiddle - lateralDistanceBack) / (sin(LASER_SIDE_BACK_ANGLE) * distanceSideBack));


        if (isFrontRelevant && isBackRelevant)
            lateralAngle = (lateralAngleFront + lateralAngleBack) / 2;
        else if (isFrontRelevant)
            lateralAngle = lateralAngleFront;
        else
            lateralAngle = lateralAngleBack;

        if (menorDistancia < BUG2_WALL_DISTANCE - ((goingToWallDistance) ? BUG2_WALL_TOLERANCE / 2 : BUG2_WALL_TOLERANCE) ||
            menorDistancia > BUG2_WALL_DISTANCE + ((goingToWallDistance) ? BUG2_WALL_TOLERANCE / 2 : BUG2_WALL_TOLERANCE))
        {
            goingToWallDistance = true;
        }
        else
        {
            goingToWallDistance = false;
        }


        if (goingToWallDistance)
        {
            // ROS_INFO("going to wall distance, menor distancia: %f", menorDistancia);
            if (distanceFrontLeft < MIN_OBJECT_DISTANCE || distanceFrontRight < MIN_OBJECT_DISTANCE)
                vel.linear.x = 0;
            else if(isBackRelevant && isFrontRelevant)
                vel.linear.x = 0.5;
            else
                vel.linear.x = MIN_ANGULAR_VELOCITY;
            vel.angular.z = velocityToAngle(((menorDistancia < BUG2_WALL_DISTANCE) ? -M_PI / 10 : M_PI / 10) + lateralAngle);
        }
        else
        {
            vel.linear.x = MAX_LINEAR_VELOCITY;

            if (fabs(lateralAngle) < YAW_TOLERANCE)
                // Ajusta o angulo bem devagar
                vel.angular.z = MIN_ANGULAR_VELOCITY * lateralAngle / YAW_TOLERANCE;
            else
                vel.angular.z = velocityToAngle(lateralAngle);
        }

        // Reduz a velocidade linear nas curvas
        vel.linear.x *= 1 - (fabs(vel.angular.z) / MAX_ANGULAR_VELOCITY);

        // ROS_INFO("Angle: %f", lateralAngle);
    }
    else
    {
        // Nunca deve chegar aqui, os IFs anteriores devem cobrir todas as possibilidade
        vel.linear.x = 0;
        vel.angular.z = 0;
        ROS_WARN("O calculo do algoritmo bug 2 chegou em uma condicao nao prevista.");
    }

    vel.angular.z *= ((pioneerStatus == Bug2ModeLeft) ? 1 : -1);

    // ROS_INFO("\nFront: %f \nMiddle: %f \nBack: %f", lateralDistanceFront, lateralDistanceMiddle, lateralDistanceBack);

    return vel;
}


// Retorna o angulo equivalente entre PI e -PI
double normalizeAngle(double angle)
{
    angle = fmod(angle + M_PI, 2 * M_PI);
    if (angle < 0)
        angle += 2 * M_PI;

    return angle - M_PI;
}
