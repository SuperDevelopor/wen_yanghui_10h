#ifndef PLANNER_H
#define PLANNER_H
#include "roadmap.h"
#include <iostream>
#include <map>
#include <multi_agent_planner/Agent.h>
#include <multi_agent_planner/AgentSrv.h>
#include <nav_msgs/Path.h>
#include <roadmap.h>
#include <ros/ros.h>
#include <ros/service.h>

#define INF 1e7
using namespace std;

/*
*name: TrajType
*desciption: 用于Dijkstra算法，记录各点当前的花费、父节点坐标
*/
class TrajType
{
public:
    int cost;  //起始点到current点总花费
    Point current, last; //current记录当前点坐标，last记录父节点坐标
    //重载<操作符，用于排序
    bool operator<(const TrajType &s) const { return (cost < s.cost); }
    //重载==操作符,用于寻找父节点时在vector中使用find函数
    bool operator==(const TrajType &s) const
    {
        return ((current.x == s.current.x) && (current.y == s.current.y));
    }
};

class Planner
{
public:
    Planner();
    /*
    *name: getPlan(multi_agent_planner::AgentSrvRequest &goal,
    *             multi_agent_planner::AgentSrvResponse &res)
    *description: ROS /get_plan 服务回调函数，内部使用Dijkstra算法规划寻找最短路径
    *return: void
    **/
    bool getPlan(multi_agent_planner::AgentSrvRequest &goal,
                 multi_agent_planner::AgentSrvResponse &res);
    /*
    *name: msgCallBack(const multi_agent_planner::AgentConstPtr &msg);
    *description: 订阅/agent_feedback回调函数，记录各agent当前位姿
    *return: void
    **/
    void msgCallBack(const multi_agent_planner::AgentConstPtr &msg);
    void run();

private:

    map<string, multi_agent_planner::Agent> current_pose_; //记录各agent位姿
    vector<vector<bool> > occupation_;  //预约表，记录地图个点占用情况
    RoadMap road_map_;  //地图
    ros::Subscriber subsciber_;
    ros::ServiceServer service_;
    ros::NodeHandle nh_;
    /*
    *name: getLast(vector<TrajType> &s, TrajType &goal);
    *
    *description: 在集合s中寻找goal的父节点
    *return: 若goal在节点s中返回true,否则返回false
    **/
    bool getLast(vector<TrajType> &s, TrajType &goal);
};

#endif // PLANNER_H
