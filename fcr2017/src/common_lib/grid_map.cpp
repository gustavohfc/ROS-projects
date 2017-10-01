#include <cstring>
#include <cmath>

#include "grid_map.h"
#include "odometer.h"
#include "laser_sensor.h"
#include "common.h"

// Global extern variable
GridMap gridMap;


void GridMap::init(ros::NodeHandle& nodeHandle)
{
    // Topico para publicar o mapa no RViz
    pub_rviz = nodeHandle.advertise<nav_msgs::OccupancyGrid>("map_rviz", 1);

    // Topico para enviar o mapa para o no map_saver
    pub_saver = nodeHandle.advertise<nav_msgs::OccupancyGrid>("map", 1);

    // Marca todas as celulas como indefinido
    for (int x = 0; x < GRID_MAP_MATRIX_SIZE_X; x++)
        for (int y = 0; y < GRID_MAP_MATRIX_SIZE_Y; y++)
                grid[y][x] = UNDEFINED_CELL;
}


void GridMap::update()
{
    for (double angle = laserSensor.getAngleMin(), max = laserSensor.getAngleMax(); angle < max; angle += GRID_MAP_LASER_ANGLE_STEP)
    {
        double objectDistance = laserSensor.atAngle(angle);

        // Marca celulas livres
        for (double distance = GRID_MAP_LASER_POINT_STEP;
             distance < objectDistance && distance < GRID_MAP_LASER_MAX_DISTANCE;
             distance += GRID_MAP_LASER_POINT_STEP)
        {
            setCell(angle, distance, EMPTY_CELL);
        }

        if (objectDistance < GRID_MAP_LASER_MAX_DISTANCE)
            setCell(angle, objectDistance, OCCUPIED_CELL);
    }

    sendMapTo(pub_rviz);
}


void GridMap::save()
{
    sendMapTo(pub_saver);
}


void GridMap::sendMapTo(ros::Publisher& pub)
{
    nav_msgs::OccupancyGrid map;

    map.info.resolution = GRID_MAP_RESOLUTION;
    map.info.width      = GRID_MAP_MATRIX_SIZE_X;
    map.info.height     = GRID_MAP_MATRIX_SIZE_Y;

    // Move o mapa para a posicao correta com cada quadrado centralizado na coordenada correspondente
    map.info.origin.position.x = GRID_MAP_LIMIT_NEGATIVE_X - (GRID_MAP_RESOLUTION / 2);
    map.info.origin.position.y = GRID_MAP_LIMIT_NEGATIVE_Y - (GRID_MAP_RESOLUTION / 2);

    // Copy grid map
    map.data.resize(map.info.width * map.info.height);
    std::memcpy( &(*map.data.begin()), &(grid[0][0]), map.data.size());

    pub.publish(map);
}


// Calcula qual eh a celual correspondente e altera seu valor
void GridMap::setCell(double angle, double distance, int8_t newValue)
{
    double theta = angle + odometer.getYaw();

    double absoluteX = odometer.getLaserSensorX() + cos(theta) * distance;
    double absoluteY = odometer.getLaserSensorY() + sin(theta) * distance;

    int yIndex = (absoluteY / GRID_MAP_RESOLUTION) + GRID_MAP_MATRIX_OFFSET_Y;
    int xIndex = (absoluteX / GRID_MAP_RESOLUTION) + GRID_MAP_MATRIX_OFFSET_X;

    if (grid[yIndex][xIndex] != OCCUPIED_CELL)
        grid[yIndex][xIndex] = newValue;
}