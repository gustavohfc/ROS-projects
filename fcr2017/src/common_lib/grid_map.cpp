#include <cstring>
#include <cmath>
#include <string>
#include <fstream>

#include "grid_map.h"

GridMap::GridMap(char _ID, double _center_x, double _center_y, double _tolerance_x, double _tolerance_y, ros::NodeHandle& nodeHandle, const Odometer& _odometer, const LaserSensor& _laser_sensor, const char *fcr2017_path)
    : ID(_ID), odometer(_odometer), laser_sensor(_laser_sensor), grid_x(_center_x - _tolerance_x), grid_y(_center_y - _tolerance_y),
    size_x(2 * _tolerance_x / GRID_MAP_RESOLUTION), size_y(2 * _tolerance_y / GRID_MAP_RESOLUTION)
{
    // Topic to send the map to display in RViz
    pub_rviz = nodeHandle.advertise<nav_msgs::OccupancyGrid>("map_rviz", 1);

    // Name of the file to save the grid map image
    image_file_path = std::string(fcr2017_path) + "/map_output/cic_node_" + ID + ".pgm";

    // Initialize grid
    grid = new int8_t* [size_y];
    for (int y = 0; y < size_y; y++)
    {
        grid[y] = new int8_t [size_x];

        for (int x = 0; x < size_x; x++)
        {
            grid[y][x] = UNDEFINED_CELL;
        }
    }
}



GridMap::~GridMap()
{
    for (int y = 0; y < size_y; y++)
        delete[] grid[y];

    delete[] grid;
}



void GridMap::saveImageFile()
{
    std::ofstream image;
    image.open(image_file_path.c_str(), std::ofstream::out | std::ofstream::trunc);

    if (!image.is_open())
        ROS_ERROR("Nao foi possivel abrir o arquivo para salvar a imagem (%s)", image_file_path.c_str());

    // Write header
    image << "P2\n";
    image << size_x << ' ' << size_y << '\n';
    image << EMPTY_CELL << '\n';

    // Write image
    for (int y = 0; y < size_y; y++)
    {
        for (int x = 0; x < size_x; x++)
        {
            image << (int) grid[y][x] << ' ';
        }
        image << '\n';
    }

    image.close();
}



void GridMap::updateGrid()
{
    // Mark the pioneer position as empty cells
    for  (double x = odometer.getX() - 0.5; x < odometer.getX() + 0.5; x += GRID_MAP_DISTANCE_STEP)
        for  (double y = odometer.getY() - 0.5; y < odometer.getY() + 0.5; y += GRID_MAP_DISTANCE_STEP)
            markCoordinateAs(x, y, EMPTY_CELL);
}



// Change the cell value, if the cell is out of geid range returns false
bool GridMap::markCoordinateAs(double coordinate_x, double coordinate_y, int8_t value)
{
    int index_x = (coordinate_x - grid_x) / GRID_MAP_RESOLUTION;
    int index_y = (coordinate_y - grid_y) / GRID_MAP_RESOLUTION;

    if (index_x < 0 || index_x > size_x || index_y < 0 || index_y > size_y)
        return false;

    if (grid[index_y][index_x] != OCCUPIED_CELL)
        grid[index_y][index_x] = value;

    return true;
}



void GridMap::sendToRviz()
{
    nav_msgs::OccupancyGrid map;

    map.info.resolution = GRID_MAP_RESOLUTION;
    map.info.width      = size_x;
    map.info.height     = size_y;

    // Move o mapa para a posicao correta com cada quadrado centralizado na coordenada correspondente
    map.info.origin.position.x = grid_x;
    map.info.origin.position.y = grid_y;

    // Copy grid map
    map.data.resize(map.info.width * map.info.height);
    for (int y = 0; y < size_y; y++)
        std::memcpy( &(*map.data.begin()), grid[y], size_x);

    pub_rviz.publish(map);
}


// void GridMap::update()
// {
//     for (double angle = laserSensor.getAngleMin(), max = laserSensor.getAngleMax(); angle < max; angle += GRID_MAP_LASER_ANGLE_STEP)
//     {
//         double objectDistance = laserSensor.atAngle(angle);

//         // Marca celulas livres
//         for (double distance = GRID_MAP_LASER_POINT_STEP;
//              distance < objectDistance && distance < GRID_MAP_LASER_MAX_DISTANCE;
//              distance += GRID_MAP_LASER_POINT_STEP)
//         {
//             setCell(angle, distance, EMPTY_CELL);
//         }

//         if (objectDistance < GRID_MAP_LASER_MAX_DISTANCE)
//             setCell(angle, objectDistance, OCCUPIED_CELL);
//     }

//     sendMapTo(pub_rviz);
// }


// void GridMap::save()
// {
//     sendMapTo(pub_saver);
// }


// void GridMap::sendMapTo(ros::Publisher& pub)
// {
//     nav_msgs::OccupancyGrid map;

//     map.info.resolution = GRID_MAP_RESOLUTION;
//     map.info.width      = GRID_MAP_MATRIX_SIZE_X;
//     map.info.height     = GRID_MAP_MATRIX_SIZE_Y;

//     // Move o mapa para a posicao correta com cada quadrado centralizado na coordenada correspondente
//     map.info.origin.position.x = GRID_MAP_LIMIT_NEGATIVE_X - (GRID_MAP_RESOLUTION / 2);
//     map.info.origin.position.y = GRID_MAP_LIMIT_NEGATIVE_Y - (GRID_MAP_RESOLUTION / 2);

//     // Copy grid map
//     map.data.resize(map.info.width * map.info.height);
//     std::memcpy( &(*map.data.begin()), &(grid[0][0]), map.data.size());

//     pub.publish(map);
// }


// // Calcula qual eh a celual correspondente e altera seu valor
// void GridMap::setCell(double angle, double distance, int8_t newValue)
// {
//     double theta = angle + odometer.getYaw();

//     double absoluteX = odometer.getLaserSensorX() + cos(theta) * distance;
//     double absoluteY = odometer.getLaserSensorY() + sin(theta) * distance;

//     int yIndex = (absoluteY / GRID_MAP_RESOLUTION) + GRID_MAP_MATRIX_OFFSET_Y;
//     int xIndex = (absoluteX / GRID_MAP_RESOLUTION) + GRID_MAP_MATRIX_OFFSET_X;

//     if (grid[yIndex][xIndex] != OCCUPIED_CELL)
//         grid[yIndex][xIndex] = newValue;
// }