#include "planner.h"
#include <iostream>
#include <ros/ros.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner");
    std::cout << "hello multi-agent" << std::endl;
    Planner *planner = new Planner();
    ros::spin();
    return (0);
}
