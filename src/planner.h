#ifndef PLANNER_H
#define PLANNER_H
#include "roadmap.h"
#include <iostream>
#include <map>
#include <multi_agent_planner/Agent.h>
#include <multi_agent_planner/AgentSrv.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <ros/service.h>

using namespace std;
class Planner
{
public:
    Planner();
    bool getPlan(multi_agent_planner::AgentSrvRequest &goal,
                 multi_agent_planner::AgentSrvResponse &res);

    void msgCallBack(const multi_agent_planner::AgentConstPtr &msg);

private:
    map<string, multi_agent_planner::Agent> current_pose_;
    RoadMap road_map_;
    ros::Subscriber subsciber_;
    ros::ServiceServer service_;
    ros::NodeHandle nh_;
};

#endif // PLANNER_H
