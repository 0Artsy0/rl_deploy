#include <csignal>
#include <ros/ros.h>
#include "FSM/FSM_Manager.hpp"
#include "Timer/thread_Timer.hpp"
#include <iostream>
class deploy
{
private:
    FSM_Manager FSM_Deploy;
    thread_Timer rl_Deploy_thread;

public:
    deploy(const std::string &sim_engine, const std::string &model_type)
      :FSM_Deploy(sim_engine,model_type){
        rl_Deploy_thread.start(250.0,std::bind(&FSM_Manager::run,&FSM_Deploy));
    }
  
    ~deploy(){
        rl_Deploy_thread.stop();
    }
};

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "go2_deploy");

    if (argc!=3){
        ROS_ERROR("example: ./rl_deploy (gazebo or mujoco) (torchscript ro onnx)");
        return 1;
    }

    deploy Robot_Deploy(argv[1],argv[2]);
    ros::spin();   
    return 0;
}