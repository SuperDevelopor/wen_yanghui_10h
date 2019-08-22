#include "planner.h"
#include "sampleagent.h"
#include <boost/thread.hpp>
#include <gtest/gtest.h>
#include <multi_agent_planner/Agent.h>
#include <multi_agent_planner/AgentSrv.h>
#include <ros/ros.h>

TEST(agent, first)
{
    ros::NodeHandle nh;
    ros::ServiceClient cilent;
    cilent = nh.serviceClient<multi_agent_planner::AgentSrv>("agent_1/update_goal");
    multi_agent_planner::AgentSrv srv;
    srv.request.sreial_id = "agent_1";
    srv.request.x = 2;
    srv.request.y = 5;
    srv.request.theta = 0;
    ASSERT_TRUE(cilent.call(srv));

    cilent = nh.serviceClient<multi_agent_planner::AgentSrv>("agent_2/update_goal");
    srv.request.sreial_id = "agent_2";
    srv.request.x = 6;
    srv.request.y = 3;
    srv.request.theta = 0;
    ASSERT_TRUE(cilent.call(srv));
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "multi_agent_teat_node");
    Planner *planner = new Planner();

    ros::AsyncSpinner spinner(10);
    spinner.start();

    string serial_id = "agent_1";
    Point tp;
    tp.x = 2;
    tp.y = 0;
    tp.yaw = 0;

    SampleAgent *agent1 = new SampleAgent(serial_id, tp);

    new boost::thread(boost::bind(&SampleAgent::run, agent1));

    serial_id = "agent_2";
    tp.x = 0;
    tp.y = 3;
    tp.yaw = 0;

    SampleAgent *agent2 = new SampleAgent(serial_id, tp);
    //agent->run();
    new boost::thread(boost::bind(&SampleAgent::run, agent2));
    ROS_INFO("please wait in 3 seconds...");
    sleep(1);

    int res = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return res;
}
