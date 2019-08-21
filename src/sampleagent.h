#ifndef SAMPLEAGENT_H
#define SAMPLEAGENT_H
#include <iostream>
#include <multi_agent_planner/Agent.h>
#include <multi_agent_planner/AgentSrv.h>
#include <nav_msgs/Path.h>
#include <roadmap.h>
#include <ros/ros.h>
using namespace std;
class SampleAgent
{
public:
    SampleAgent(string &serial_id, Point &start);
    bool update_goal(multi_agent_planner::AgentSrvRequest &goal,
                     multi_agent_planner::AgentSrvResponse &res);
    void run();

private:
    multi_agent_planner::Agent current_pose_;
    ros::NodeHandle nh_;
    ros::Publisher publisher_, viewer_;
    ros::ServiceServer service_;
    ros::ServiceClient cilent_;
};

#endif // SAMPLEAGENT_H
