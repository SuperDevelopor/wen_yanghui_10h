#ifndef ROADMAP_H
#define ROADMAP_H
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

using namespace std;

class Point
{
public:
    int x, y;
};

class Node : public Point
{
public:
    float yaw, radius;
    vector<Node *> adjacent;
    Node()
    {
        yaw = 0;
        radius = 0.45f;
    }
};

class RoadMap
{
public:
    vector<vector<Node> > scatter_map_;
    // vector<vector<float>> adjacent_matrix_;
    RoadMap();
};

#endif // ROADMAP_H
