#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <deque>
#include "common.h"


enum graphNodesName { A = 0, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, endGraphNodesNameEnum };


struct nodeInfo
{
    double x, y, xTolerance, yTolerance;
};


class Graph
{
private:
    double graphAdj[endGraphNodesNameEnum][endGraphNodesNameEnum];
    nodeInfo graphNodes[endGraphNodesNameEnum];

    void clearGraphAdj();
    void addCicNodes();
    void addCicEdges();
    void addBidirectionalEdge(graphNodesName node1, graphNodesName node2, double distance);
    void addEdge(graphNodesName from, graphNodesName to, double distance);
    void addNodeInfo(graphNodesName node, double x, double y, double xTolerance, double yTolerance);


public:
    Graph();
    graphNodesName getCurrentNode();
    void findPathDijkstra(std::deque<Position>& goals);
};

#endif