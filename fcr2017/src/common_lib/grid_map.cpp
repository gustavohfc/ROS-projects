#include <cstring>

#include "grid_map.h"

// Global extern variable
GridMap gridMap;


void GridMap::init(ros::NodeHandle& nodeHandle)
{
    pub = nodeHandle.advertise<nav_msgs::OccupancyGrid>("map", 1);

    for (int x = 0; x < GRID_MAP_MATRIX_SIZE_X; x++)
    {
        for (int y = 0; y < GRID_MAP_MATRIX_SIZE_Y; y++)
        {
            if (x == GRID_MAP_MATRIX_OFFSET_X && y == GRID_MAP_MATRIX_OFFSET_Y)
                grid[y][x] = EMPTY_CELL_VALUE;
            else
                grid[y][x] = OCCUPIED_CELL_VALUE;
        }
    }
}


void GridMap::update()
{
    sendMapToRviz();
}


void GridMap::sendMapToRviz()
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