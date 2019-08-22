#ifndef ROADMAP_H
#define ROADMAP_H
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

using namespace std;

#define SQURE 11 //SQURE x SQURE的方阵
class Point
{
public:
    int x, y;
    float yaw;
};

//地图节点
class Node : public Point
{
public:
    vector<Node *> adjacent;//存放相邻节点指针
	
    /*
    *func: bool isAdjoin(const int x, const int y)
    *param: x y 坐标值
    *description: 判断点（x,y）是否与当前点相邻
    *return: 相邻返回true，否则返回false
    **/
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
