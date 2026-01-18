#pragma once

#include <iostream>
#include <ros/ros.h>
#include <robot_msgs/MotorCommand.h>
#include <robot_msgs/MotorState.h>
#include <robot_msgs/RobotState.h>
#include <robot_msgs/RobotCommand.h>
#include <sensor_msgs/Imu.h>
#include <boost/bind.hpp>
#include <thread>
#include <vector>

#include "hardware_interface.hpp"

#define interfaze_rate 500

class mujoco_interface : public hardware_interface
{
private:
    ros::NodeHandle _nh;
    ros::Publisher robot_command_pub;    // 机器人命令发布者
    ros::Subscriber robot_state_sub;    // 机器人状态订阅者

    std::thread mujoco_control_thread; // 控制线程

    std::vector<std::string> robot_State_topic = {
        "/" + _robot_param.robot_name + "_mujoco/robot_state"};

    std::vector<std::string> robot_Command_topic = {
        "/" + _robot_param.robot_name + "_mujoco/robot_command"};

public:
    mujoco_interface(const std::string &hardware_name="mujoco_interface") : hardware_interface(hardware_name) // 初始化硬件接口
    {
        robot_command_pub = _nh.advertise<robot_msgs::RobotCommand>(robot_Command_topic[0], 10); // 发布机器人命令
        robot_state_sub = _nh.subscribe<robot_msgs::RobotState>(robot_State_topic[0], 10, &mujoco_interface::robot_state_callback, this); // 订阅机器人状态

        // 启动控制线程
        mujoco_control_thread = std::thread(&mujoco_interface::mujoco_control, this);
    }

    void robot_state_callback(const robot_msgs::RobotState::ConstPtr &msg)// 机器人状态回调函数
    {
        // 更新关节位置和速度
        for (int i = 0; i < dof_nums; i++)
        {
            pos[i] = (*msg).motor_state[i].q; // 位置
            vel[i] = (*msg).motor_state[i].dq; // 速度
        }

        // 更新IMU数据
        std::vector<float> quaternion = {(*msg).imu.quaternion[0], (*msg).imu.quaternion[1], (*msg).imu.quaternion[2], (*msg).imu.quaternion[3]};
        gravity_projection_compute(quaternion); // 计算重力投影
        angle_vel = {(*msg).imu.gyroscope[0], (*msg).imu.gyroscope[1], (*msg).imu.gyroscope[2]}; // 角速度
    }

    void cmd_publish() // 发布命令函数
    {
        robot_msgs::RobotCommand robot_command;
        for (int i = 0; i < dof_nums; i++)
        {
            robot_command.motor_command[i].q = cmd_pos[i]; // 位置
            robot_command.motor_command[i].dq = cmd_vel[i]; // 速度
            robot_command.motor_command[i].kp = kp[i]; // 位置比例增益
            robot_command.motor_command[i].kd = kd[i]; // 速度微分增益
            robot_command.motor_command[i].tau = cmd_torque[i]; // 力矩
        }
        robot_command_pub.publish(robot_command);
    }

    void mujoco_control() // 控制函数
    {
        ros::Rate loop_rate(interfaze_rate); // 500Hz控制频率
        while (ros::ok())
        {
            cmd_publish(); // 发布命令
            loop_rate.sleep();
            ros::spinOnce();
        }
    }
    
    ~mujoco_interface()
    {

    }
};
