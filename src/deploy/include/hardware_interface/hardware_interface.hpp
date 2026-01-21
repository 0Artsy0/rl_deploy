#pragma once

#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <thread>
#include "robot_param.hpp"

class hardware_interface
{
private:
    std::string hardware_name;
public:
    robot_param _robot_param; // 机器人参数

    //关节状态信息
    std::vector<float> pos;//关节位置
    std::vector<float> vel;//关节速度

    //关节控制指令
    std::vector<float> cmd_pos;//关节位置指令
    std::vector<float> cmd_vel;//关节速度指令
    std::vector<float> cmd_torque;//关节力矩指令
    std::vector<float> kp;//关节位置指令
    std::vector<float> kd;//关节速度指令

    //imu状态信息
    std::vector<float> gravity_projection;//重力投影
    std::vector<float> angle_vel;//机身角速度

    bool running = false;
    bool data_received = false;
    
    hardware_interface(const std::string &hardware_name=""):_robot_param()
    {
        this->hardware_name=hardware_name;
        
        pos.resize(_robot_param.dof_nums,0.0f);
        vel.resize(_robot_param.dof_nums,0.0f);

        cmd_pos.resize(_robot_param.dof_nums,0.0f);
        cmd_vel.resize(_robot_param.dof_nums,0.0f);
        cmd_torque.resize(_robot_param.dof_nums,0.0f);
        kp.resize(_robot_param.dof_nums,0.0f);
        kd.resize(_robot_param.dof_nums,0.0f);
        gravity_projection.resize(3,0.0f);
        angle_vel.resize(3,0.0f);
    }
    virtual ~hardware_interface()=default;

    std::vector<float> get_joint_pos()//获取关节位置
    {
        return pos;
    }

    std::vector<float> get_joint_vel()//获取关节速度
    {
        return vel;
    }

    std::string get_hardware_name()//获取硬件名称
    {
        return hardware_name;
    }

    void command_set(std::vector<float> cmd_pos={0.0}, std::vector<float> cmd_vel={0.0}, std::vector<float> cmd_torque={0.0}, std::vector<float> kp={0.0}, std::vector<float> kd={0.0})
    {
        this->cmd_pos=cmd_pos;
        this->cmd_vel=cmd_vel;
        this->cmd_torque=cmd_torque;
        this->kp=kp;
        this->kd=kd;
    }

    void gravity_projection_compute(std::vector<float> quaternion)//计算重力投影
    {
        //四元数转重力投影，初始状态[1,0,0,0]对应重力投影[0,0,-1]
        gravity_projection[0] = 2*(quaternion[1]*quaternion[3] - quaternion[0]*quaternion[2]);
        gravity_projection[1] = 2*(quaternion[0]*quaternion[1] + quaternion[2]*quaternion[3]);
        gravity_projection[2] = -(quaternion[0]*quaternion[0] - quaternion[1]*quaternion[1] - quaternion[2]*quaternion[2] + quaternion[3]*quaternion[3]);
    }
  
};