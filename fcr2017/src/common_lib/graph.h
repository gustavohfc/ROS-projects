#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include "common.h"
#include "odometer.h"

struct Node;


struct Edge
{
    Edge(Node *_dest_node, double _cost)
        : dest_node(_dest_node), cost(_cost) {}
    Node *dest_node;
    double cost;
};


struct Node
{
    Node(char _ID, double _center_x, double _center_y, double _tolerance_x, double _tolerance_y)
        : ID(_ID), center(Position(_ID, _center_x, _center_y)), tolerance_x(_tolerance_x), tolerance_y(_tolerance_y) {}

    char ID;
    Position center;
    double tolerance_x, tolerance_y;
    std::vector<Edge> edges;
};


class Graph
{
private:
    std::vector<Node> nodes;
    const Odometer& odometer;

    void addNode(char ID, double center_x, double center_y, double tolerance_x, double tolerance_y);
    void addBidirectionalEdge(char ID_node_1, char ID_node_2, double cost);
    void addEdge(char ID_node_1, char ID_node_2, double cost);
    int getNodeIndex(char ID);


public:
    Graph(const char *file_name, const Odometer& _odometer);
    Node* getNode(char ID, bool must_exist = true);
    Node* getCurrentNode();
    std::vector<Position> Dijkstra(char dest_node_ID);
    // graphNodesName getCurrentNode();
    // void findPathDijkstra(std::deque<Position>& goals);
};

#endif