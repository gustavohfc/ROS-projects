#include <ros/ros.h>

#include "graph.h"
#include "odometry.h"


Graph::Graph()
{
    clearGraphAdj();
    addCicNodes();
    addCicEdges();
}


void Graph::clearGraphAdj()
{
    for (int i = A; i < endGraphNodesNameEnum; i++)
        for (int j = A; j < endGraphNodesNameEnum; j++)
            graphAdj[i][j] = -1;
}


void Graph::addCicNodes()
{
    double d_quad = 1.30 + 0.1; // 0.2 in all
    double y_mid = 7.14 + 0.1;
    double x_left = 10.99 + 0.1;
    double x_mid_right = 6.65 + 0.1;
    double x_corridor = 3.47 + 0.1;
     
    addNodeInfo(A, -27.38, 17.67, d_quad , d_quad );
    addNodeInfo(B, -27.20, 8.67, d_quad , y_mid );
    addNodeInfo(C, -27.18, -.19, d_quad , d_quad );
    addNodeInfo(D, -14.09, 17.58, x_left , d_quad );
    addNodeInfo(E, -14.01, -.17, x_left , d_quad );
    addNodeInfo(F,  -.96, 17.60, d_quad , d_quad );
    addNodeInfo(G,  -.78, 8.75, d_quad , y_mid );
    addNodeInfo(H,  -.50, -.22, d_quad , d_quad );
    addNodeInfo(I,  7.74, 17.76, x_mid_right , d_quad );
    addNodeInfo(J,  8.05, -.22, x_mid_right , d_quad );
    addNodeInfo(K,  19.07, 17.83, x_corridor , d_quad );
    addNodeInfo(L,  19.12, 8.69, x_corridor , y_mid );
    addNodeInfo(M,  18.98, -.27, x_corridor , d_quad );
    addNodeInfo(N,  30.62, 17.74, x_mid_right , d_quad );
    addNodeInfo(O,  30.20, -.23, x_mid_right , d_quad );
    addNodeInfo(P,  38.71, 17.72, d_quad , d_quad );
    addNodeInfo(Q,  38.87, 8.86, d_quad , y_mid );
    addNodeInfo(R,  38.61, -.23, d_quad , d_quad ); //R
}


void Graph::addCicEdges()
{
    addBidirectionalEdge(A, B, 18);
    addBidirectionalEdge(A, D, 26);
    addBidirectionalEdge(D, F, 29);
    addBidirectionalEdge(F, G, 19);
    addBidirectionalEdge(F, I, 20);
    addBidirectionalEdge(I, K, 21);
    addBidirectionalEdge(K, L, 18);
    addBidirectionalEdge(K, N, 24);
    addBidirectionalEdge(N, P, 18);
    addBidirectionalEdge(P, Q, 18);
    addBidirectionalEdge(Q, R, 18);
    addBidirectionalEdge(R, O, 18);
    addBidirectionalEdge(O, M, 24);
    addBidirectionalEdge(M, L, 18);
    addBidirectionalEdge(M, J, 23);
    addBidirectionalEdge(J, H, 20);
    addBidirectionalEdge(H, G, 19);
    addBidirectionalEdge(H, E, 28);
    addBidirectionalEdge(E, C, 27);
    addBidirectionalEdge(C, B, 19);
}


void Graph::addBidirectionalEdge(graphNodesName node1, graphNodesName node2, double distance)
{
    addEdge(node1, node2, distance);
    addEdge(node2, node1, distance);
}


void Graph::addEdge(graphNodesName from, graphNodesName to, double distance)
{
    graphAdj[from][to] = distance;
}


void Graph::addNodeInfo(graphNodesName node, double x, double y, double xTolerance, double yTolerance)
{
    graphNodes[node].x = x;
    graphNodes[node].y = y;
    graphNodes[node].xTolerance = xTolerance;
    graphNodes[node].yTolerance = yTolerance;
}


graphNodesName Graph::getCurrentNode()
{
    for (int i = A; i < endGraphNodesNameEnum; i++)
    {
        if (fabs(pioneerPosition.x - graphNodes[i].x) < graphNodes[i].xTolerance && 
            fabs(pioneerPosition.y - graphNodes[i].y) < graphNodes[i].yTolerance)
        {
            return (graphNodesName) i;
        }
    }

    ROS_ERROR("Localizacao do Pioneer fora do grafo");
    exit(1);
}


void Graph::findPathDijkstra(std::deque<Position>& goals)
{
    bool visited[endGraphNodesNameEnum];
    double dist[endGraphNodesNameEnum];
    graphNodesName parent[endGraphNodesNameEnum];

    graphNodesName startNode = getCurrentNode();

    for (int i = 0; i <  endGraphNodesNameEnum; ++i)
    {
        parent[i] = endGraphNodesNameEnum;
        visited[i] = false;
        dist[i] = std::numeric_limits<double>::max();
    }

    dist[startNode]=0;

    for(int i = 0; i < endGraphNodesNameEnum; i++)
    {
        int node = -1;

        for (int j = 0; j < endGraphNodesNameEnum; ++j) 
        {
            if(!visited[j] && (node == -1 || dist[j] < dist[node]))
            {
                node=j;
            }
        }
    
        visited[node] = true;

        if(dist[node] == std::numeric_limits<double>::max())
        {   
            break;
        }
    
        for (int j = 0; j < endGraphNodesNameEnum; ++j)
        {
            if(graphAdj[node][j] && dist[j] > dist[node] + graphAdj[node][j])
            {   
                dist[j] = dist[node] + graphAdj[node][j];
                parent[j] = (graphNodesName) node;
            }
        }
    }


    graphNodesName temp = P;
    do
    {
        ROS_INFO("%d", temp);
        temp = parent[temp];
        break;
    }while(temp != endGraphNodesNameEnum);
}