#include "PPO/Torchscript/interface_torchscript.hpp"
#include <iostream>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");

    InterfaceTorchscript _Interface_torchscript("/home/lym/zhang/deploy/src/policy/Lite3/policy.pt");

    ros::Rate rate(100);
    while (ros::ok())
    {
        _Interface_torchscript.push_forward();
        rate.sleep();
    }

    std::cout << "test" << std::endl;
    return 0;
}