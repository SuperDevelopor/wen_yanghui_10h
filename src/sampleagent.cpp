#include "sampleagent.h"

SampleAgent::SampleAgent(string &serial_id, Point &start)
{
    current_pose_.sreial_id = serial_id;
    current_pose_.x = start.x;
    current_pose_.y = start.y;
    current_pose_.theta = start.yaw;

    publisher_ = nh_.advertise<multi_agent_planner::Agent>("agent_feedback", 10);
    viewer_ = nh_.advertise<nav_msgs::Path>(serial_id + "/path_view", 10);
    service_ = nh_.advertiseService(serial_id + "/update_goal", &SampleAgent::update_goal, this);
    cilent_ = nh_.serviceClient<multi_agent_planner::AgentSrv>("get_plan");
}

bool SampleAgent::update_goal(multi_agent_planner::AgentSrvRequest &goal,
                              multi_agent_planner::AgentSrvResponse &res)
{
    //error check
    ROS_INFO("recieved a goal plan.");
    if (goal.x > SQURE || goal.x < 0 || goal.y > SQURE || goal.y < 0) {
        ROS_ERROR("invalid goal!!");
        return false;
    }

    multi_agent_planner::AgentSrv srv;
    srv.request = goal;
    srv.request.sreial_id = current_pose_.sreial_id;
    if (cilent_.call(srv)) {
        viewer_.publish(srv.response.path);
        res = srv.response;
    } else {
        ROS_ERROR("Failed to call service 'get_plan'");
    }
    return (true);
}

void SampleAgent::run()
{
    ros::Rate loop(10);
    while (ros::ok()) {
        publisher_.publish(current_pose_);
        ros::spinOnce();
        loop.sleep();
    }
}
