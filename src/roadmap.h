#ifndef ROADMAP_H
#define ROADMAP_H
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

using namespace std;

#define SQURE 11
class Point
{
public:
    int x, y;
    float yaw;
};

class Node : public Point
{
public:
    vector<Node *> adjacent;
    bool isAdjoin(const int x, const int y)
    {
        for (int i = 0; i < adjacent.size(); i++) {
            if ((adjacent[i]->x == x) && (adjacent[i]->y == y)) {
                return (true);
            }
        }
        return (false);
    }
    Node() { yaw = 0; }
};

class RoadMap
{
public:
    vector<vector<Node> > scatter_map_;
    // vector<vector<float>> adjacent_matrix_;
    RoadMap();
};

#endif // ROADMAP_H
