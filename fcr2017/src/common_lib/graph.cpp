#include <ros/ros.h>
#include <fstream>
#include <string>
#include <stdlib.h>

#include "graph.h"


Graph::Graph(const char *file_name, const Odometer& _odometer)
    : odometer(_odometer)
{
    std::ifstream graph_file(file_name);
    std::string line;
    int line_count = 1;

    if (!graph_file.is_open())
    {
        ROS_ERROR("Nao foi possivel abrir o arquivo %s", file_name);
        exit(EXIT_FAILURE);
    }

    // Read the graph definition file
    while (std::getline(graph_file, line))
    {
        bool bad_line_parse = false;

        if (line.empty())
            continue;

        std::istringstream line_stream(line);

        std::string line_type;
        line_stream >> line_type;

        if (line_type.compare("NODE_INFO") == 0)
        {
            char ID;
            double center_x, center_y, tolerance_x, tolerance_y;

            line_stream >> ID >> center_x >> center_y >> tolerance_x >> tolerance_y;

            if (line_stream.fail())
                bad_line_parse = true;
            else
                addNode(ID, center_x, center_y, tolerance_x, tolerance_y);
        }
        else if (line_type.compare("EDGE_INFO") == 0)
        {
            char ID_node_1, ID_node_2;
            double cost;

            line_stream >> ID_node_1 >> ID_node_2 >> cost;

            if (line_stream.fail())
                bad_line_parse = true;
            else
                addBidirectionalEdge(ID_node_1, ID_node_2, cost);
        }
        else if (line_type.compare("###") == 0)
        {} // It's a commented line, so do nothing
        else
            bad_line_parse = true;

        if (bad_line_parse)
        {
            ROS_ERROR("Erro no parser da linha %d do arquivo %s (%s)", line_count, file_name, line.c_str());
            exit(EXIT_FAILURE);
        }

        line_count++;
    }

    graph_file.close();
}



void Graph::addNode(char ID, double center_x, double center_y, double tolerance_x, double tolerance_y)
{
    // Check if the graph already has a node with the same ID
    if (getNode(ID, false) != NULL)
    {
        ROS_ERROR("Tentando inserir ID repetido, ID = %c (%d)", ID, ID);
        exit(EXIT_FAILURE);
    }

    nodes.push_back(Node(ID, center_x, center_y, tolerance_x, tolerance_y));
}



void Graph::addBidirectionalEdge(char ID_node_1, char ID_node_2, double cost)
{
    addEdge(ID_node_1, ID_node_2, cost);
    addEdge(ID_node_2, ID_node_1, cost);
}



void Graph::addEdge(char ID_node_from, char ID_node_to, double cost)
{
    getNode(ID_node_from)->edges.push_back(Edge(getNode(ID_node_to), cost));
}



Node* Graph::getNode(char ID, bool must_exist)
{
    for(std::vector<Node>::iterator it = nodes.begin(); it != nodes.end(); ++it)
    {
        if (it->ID == ID)
            return &(*it);
    }

    if (must_exist)
    {
        ROS_ERROR("Tentando aceesar nó que não existe no grafo, ID = %c (%d)", ID, ID);
        exit(EXIT_FAILURE);
    }
    else
        return NULL;
}



int Graph::getNodeIndex(char ID)
{
    for(std::vector<Node>::iterator it = nodes.begin(); it != nodes.end(); ++it)
    {
        if (it->ID == ID)
            return it - nodes.begin();
    }
}



Node* Graph::getCurrentNode()
{
    for(std::vector<Node>::iterator it = nodes.begin(); it != nodes.end(); ++it)
    {
        if (odometer.getX() < it->center.x + it->tolerance_x &&
            odometer.getX() > it->center.x - it->tolerance_x &&
            odometer.getY() < it->center.y + it->tolerance_y &&
            odometer.getY() > it->center.y - it->tolerance_y)
        {
            return &(*it);
        }
    }

    return NULL;
}



std::vector<Position> Graph::Dijkstra(char dest_node_ID)
{
    bool all_nodes_visited = false;

    Node* initial_node = getNode('H');
    // Node* initial_node = getCurrentNode();
    if (initial_node == NULL)
    {
        ROS_ERROR("O Pionneer esta fora do grafo");
        exit(EXIT_FAILURE);
    }

    std::vector<double> distance(nodes.size(), std::numeric_limits<double>::infinity());
    std::vector<int> previous(nodes.size(), -1);
    std::vector<bool> visited(nodes.size(), false);

    distance[ getNodeIndex(initial_node->ID) ] = 0;

    while (!all_nodes_visited)
    {
        int node_index;
        double node_distance = std::numeric_limits<double>::infinity();

        // Find the node with least distance
        for (int i = 0; i < nodes.size(); i++)
        {
            if (!visited[i] && distance[i] < node_distance)
            {
                node_index = i;
                node_distance = distance[i];
            }
        }

        visited[node_index] = true;

        // Check if the path to final destination was found
        if (nodes[node_index].ID == dest_node_ID)
        {
            std::vector<Position> path;

            while (previous[node_index] != -1)
            {
                path.insert(path.begin(), nodes[node_index].center);
                node_index = previous[node_index];
            }

            return path;
        }


        if (node_distance == std::numeric_limits<double>::infinity())
        {
            all_nodes_visited = true;
        }
        else
        {
            for (int i = 0; i < nodes[node_index].edges.size(); i++)
            {
                int neighbor_index = getNodeIndex(nodes[node_index].edges[i].dest_node->ID);

                if (distance[node_index] + nodes[node_index].edges[i].cost < distance[neighbor_index])
                {
                    distance[neighbor_index] = distance[node_index] + nodes[node_index].edges[i].cost;
                    previous[neighbor_index] = node_index;
                }
            }
        }
    }

    ROS_ERROR("a");
    exit(EXIT_FAILURE);
}

