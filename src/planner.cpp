#include "planner.h"

Planner::Planner()
{
    subsciber_ = nh_.subscribe<multi_agent_planner::Agent>("agent_feedback",
                                                           10,
                                                           &Planner::msgCallBack,
                                                           this);
    service_ = nh_.advertiseService("get_plan", &Planner::getPlan, this);
}

bool Planner::getPlan(multi_agent_planner::AgentSrvRequest &goal,
                      multi_agent_planner::AgentSrvResponse &res)
{
    return (true);
}

void Planner::msgCallBack(const multi_agent_planner::AgentConstPtr &msg)
{
    map<string, multi_agent_planner::Agent>::iterator it;
    it = current_pose_.find(msg->sreial_id);
    if (it == current_pose_.end()) {
        current_pose_.insert(make_pair(msg->sreial_id, *msg));
    } else {
        current_pose_[msg->sreial_id] = *msg;
    }
}
