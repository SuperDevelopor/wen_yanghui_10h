#include "sampleagent.h"
#include <iostream>
#include <roadmap.h>
#include <ros/ros.h>
int main(int argc, char **argv)
{
    if (argc != 5) {
        ROS_ERROR("error usage!! \r\n"
                  "eg: ./agent_node serial_id x y theta");
        exit(1);
    }

    ros::init(argc, argv, argv[1]);
    string serial_id = argv[1];
    ROS_INFO("sample %s running.", serial_id.c_str());
    Point tp;
    tp.x = std::atoi(argv[2]);   //static_cast<int>(*argv[2]);
    tp.y = std::atoi(argv[3]);   //static_cast<int>(*argv[3]);
    tp.yaw = std::atof(argv[4]); //static_cast<float>(*argv[4]);
    ROS_INFO("init pose: (%d, %d, %f)", tp.x, tp.y, tp.yaw);

    //serial_id.push_back(argv[1]);
    SampleAgent *agent = new SampleAgent(serial_id, tp);
    agent->run();
    printf("\r\nprogram exit.\r\n");
    return (0);
}
