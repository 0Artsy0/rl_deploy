#pragma once

#include <iostream>
#include <ros/ros.h>
#include <robot_msgs/MotorCommand.h>
#include <robot_msgs/MotorState.h>
#include <sensor_msgs/Imu.h>
#include <boost/bind.hpp>
#include <thread>
#include <vector>

#include "hardware_interface.hpp"

#define interfaze_rate 500

class gazebo_interface : public hardware_interface
{
private:
    ros::NodeHandle _nh;

    std::vector<ros::Subscriber> _sub;
    std::vector<ros::Publisher> _pub;
    
    ros::Subscriber _imu_sub; // 订阅imu状态

    std::thread _gazebo_control_thread; // 控制线程

    std::vector<std::string> _rec_topic = {// 订阅话题
                                           "/" + _robot_param.robot_name + "_gazebo/" + _robot_param.joint_name[0] + "_controller/state",
                                           "/" + _robot_param.robot_name + "_gazebo/" + _robot_param.joint_name[1] + "_controller/state",
                                           "/" + _robot_param.robot_name + "_gazebo/" + _robot_param.joint_name[2] + "_controller/state",
                                           "/" + _robot_param.robot_name + "_gazebo/" + _robot_param.joint_name[3] + "_controller/state",
                                           "/" + _robot_param.robot_name + "_gazebo/" + _robot_param.joint_name[4] + "_controller/state",
                                           "/" + _robot_param.robot_name + "_gazebo/" + _robot_param.joint_name[5] + "_controller/state",
                                           "/" + _robot_param.robot_name + "_gazebo/" + _robot_param.joint_name[6] + "_controller/state",
                                           "/" + _robot_param.robot_name + "_gazebo/" + _robot_param.joint_name[7] + "_controller/state",
                                           "/" + _robot_param.robot_name + "_gazebo/" + _robot_param.joint_name[8] + "_controller/state",
                                           "/" + _robot_param.robot_name + "_gazebo/" + _robot_param.joint_name[9] + "_controller/state",
                                           "/" + _robot_param.robot_name + "_gazebo/" + _robot_param.joint_name[10] + "_controller/state",
                                           "/" + _robot_param.robot_name + "_gazebo/" + _robot_param.joint_name[11] + "_controller/state"};

    std::vector<std::string> _pub_topic = {// 发布话题
                                           "/" + _robot_param.robot_name + "_gazebo/" + _robot_param.joint_name[0] + "_controller/command",
                                           "/" + _robot_param.robot_name + "_gazebo/" + _robot_param.joint_name[1] + "_controller/command",
                                           "/" + _robot_param.robot_name + "_gazebo/" + _robot_param.joint_name[2] + "_controller/command",
                                           "/" + _robot_param.robot_name + "_gazebo/" + _robot_param.joint_name[3] + "_controller/command",
                                           "/" + _robot_param.robot_name + "_gazebo/" + _robot_param.joint_name[4] + "_controller/command",
                                           "/" + _robot_param.robot_name + "_gazebo/" + _robot_param.joint_name[5] + "_controller/command",
                                           "/" + _robot_param.robot_name + "_gazebo/" + _robot_param.joint_name[6] + "_controller/command",
                                           "/" + _robot_param.robot_name + "_gazebo/" + _robot_param.joint_name[7] + "_controller/command",
                                           "/" + _robot_param.robot_name + "_gazebo/" + _robot_param.joint_name[8] + "_controller/command",
                                           "/" + _robot_param.robot_name + "_gazebo/" + _robot_param.joint_name[9] + "_controller/command",
                                           "/" + _robot_param.robot_name + "_gazebo/" + _robot_param.joint_name[10] + "_controller/command",
                                           "/" + _robot_param.robot_name + "_gazebo/" + _robot_param.joint_name[11] + "_controller/command"};

    std::vector<std::string> imu_topic = {
                                            "/trunk_imu"};
public:
    gazebo_interface(const std::string &hardware_name="gazebo_interface") : hardware_interface(hardware_name) // 初始化
    {
        _sub.resize(_robot_param.dof_nums); // 订阅关节状态
        _pub.resize(_robot_param.dof_nums); // 发布关节控制命令

        for (int i = 0; i < _robot_param.dof_nums; i++)
        {
            _sub[i] = _nh.subscribe<robot_msgs::MotorState>(_rec_topic[i], 10, boost::bind(&gazebo_interface::motor_state_callback, this, _1, i)); // 订阅电机状态
            _pub[i] = _nh.advertise<robot_msgs::MotorCommand>(_pub_topic[i], 10);                                                                  // 发布电机控制命令
        }
        _imu_sub = _nh.subscribe<sensor_msgs::Imu>(imu_topic[0], 10, &gazebo_interface::imu_callback, this); // 订阅imu状态
        
        _gazebo_control_thread = std::thread(&gazebo_interface::gazebo_control, this); // 启动控制线程
    }

    void gazebo_interface::motor_state_callback(const robot_msgs::MotorState::ConstPtr &msg, int index) // 电机状态回调函数
    {
        pos[index] = (*msg).q;  // 位置
        vel[index] = (*msg).dq; // 速度
    }

    void imu_callback(const sensor_msgs::Imu::ConstPtr &msg) // imu状态回调函数
    {
        std::vector<float> quaternion = {static_cast<float>((*msg).orientation.w), static_cast<float>((*msg).orientation.x), static_cast<float>((*msg).orientation.y), static_cast<float>((*msg).orientation.z)};
        gravity_projection_compute(quaternion); // 计算重力投影
        angle_vel = {static_cast<float>((*msg).angular_velocity.x), static_cast<float>((*msg).angular_velocity.y), static_cast<float>((*msg).angular_velocity.z)}; // 角速度
    }

    void cmd_publish() // 发布电机控制命令
    {
        for (int i = 0; i < _robot_param.dof_nums; i++)
        {
            robot_msgs::MotorCommand cmd;
            cmd.q = cmd_pos[i];
            cmd.dq = cmd_vel[i];
            cmd.tau = cmd_torque[i];
            cmd.kp = kp[i];
            cmd.kd = kd[i];
            _pub[i].publish(cmd);
        }
    }

    void gazebo_control() // 控制线程函数
    {
        ros::Rate rate(interfaze_rate);
        while (ros::ok())
        {
            cmd_publish();
            ros::spinOnce();
            rate.sleep();
        }
    }

    ~gazebo_interface()
    {
        if (_gazebo_control_thread.joinable())
        {
            _gazebo_control_thread.join();
        }
    }
};

