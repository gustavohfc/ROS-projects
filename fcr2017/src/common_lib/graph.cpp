#include <ros/ros.h>
#include <fstream>
#include <string>
#include <stdlib.h>

#include "graph.h"


Graph::Graph(const char *file_name)
{
    std::ifstream graph_file(file_name);
    std::string line;
    int line_count = 1;

    if (!graph_file.is_open())
    {
        ROS_ERROR("Nao foi possivel abrir o arquivo %s", file_name);
        exit(EXIT_FAILURE);
    }

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









// graphNodesName Graph::getCurrentNode()
// {
//     for (int i = A; i < endGraphNodesNameEnum; i++)
//     {
//         if (fabs(pioneerPosition.x - graphNodes[i].x) < graphNodes[i].xTolerance && 
//             fabs(pioneerPosition.y - graphNodes[i].y) < graphNodes[i].yTolerance)
//         {
//             return (graphNodesName) i;
//         }
//     }

//     ROS_ERROR("Localizacao do Pioneer fora do grafo");
//     exit(EXIT_FAILURE);
// }


// void Graph::findPathDijkstra(std::deque<Position>& goals)
// {
//     bool visited[endGraphNodesNameEnum];
//     double dist[endGraphNodesNameEnum];
//     graphNodesName parent[endGraphNodesNameEnum];

//     graphNodesName startNode = getCurrentNode();

//     for (int i = 0; i <  endGraphNodesNameEnum; ++i)
//     {
//         parent[i] = endGraphNodesNameEnum;
//         visited[i] = false;
//         dist[i] = std::numeric_limits<double>::max();
//     }

//     dist[startNode]=0;

//     for(int i = 0; i < endGraphNodesNameEnum; i++)
//     {
//         int node = -1;

//         for (int j = 0; j < endGraphNodesNameEnum; ++j) 
//         {
//             if(!visited[j] && (node == -1 || dist[j] < dist[node]))
//             {
//                 node=j;
//             }
//         }
    
//         visited[node] = true;

//         if(dist[node] == std::numeric_limits<double>::max())
//         {   
//             break;
//         }
    
//         for (int j = 0; j < endGraphNodesNameEnum; ++j)
//         {
//             if(graphAdj[node][j] && dist[j] > dist[node] + graphAdj[node][j])
//             {   
//                 dist[j] = dist[node] + graphAdj[node][j];
//                 parent[j] = (graphNodesName) node;
//             }
//         }
//     }


//     graphNodesName temp = P;
//     do
//     {
//         ROS_INFO("%d", temp);
//         temp = parent[temp];
//         break;
//     }while(temp != endGraphNodesNameEnum);
// }