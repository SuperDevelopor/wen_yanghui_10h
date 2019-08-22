#include "planner.h"
#include <iostream>
#include <ros/ros.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner");
    std::cout << "multi-agent planner running" << std::endl;
    Planner *planner = new Planner();
    planner->run();
    return (0);
}
