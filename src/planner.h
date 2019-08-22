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
class TrajType
{
public:
    int cost;
    Point current, last;
    bool operator<(const TrajType &s) const { return (cost < s.cost); }
    bool operator==(const TrajType &s) const
    {
        return ((current.x == s.current.x) && (current.y == s.current.y));
    }
};

class Planner
{
public:
    Planner();
    bool getPlan(multi_agent_planner::AgentSrvRequest &goal,
                 multi_agent_planner::AgentSrvResponse &res);

    void msgCallBack(const multi_agent_planner::AgentConstPtr &msg);
    void run();

private:
    map<string, multi_agent_planner::Agent> current_pose_;
    vector<vector<bool> > occupation_;
    RoadMap road_map_;
    ros::Subscriber subsciber_;
    ros::ServiceServer service_;
    ros::NodeHandle nh_;

    bool getLast(vector<TrajType> &s, TrajType &goal);
};

#endif // PLANNER_H
