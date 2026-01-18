#pragma once

#include <thread>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include "utils.hpp"
#include "Timer/thread_Timer.hpp"
#include <robot_msgs/RobotState.h>
#include <robot_msgs/RobotCommand.h>
#include "usr_interface/Joy_interface.hpp"
#include "hardware_interface/gazebo_interface.hpp"
#include "hardware_interface/mujoco_interface.hpp"
#include "hardware_interface/hardware_interface.hpp"

enum class RobotState_List
{
    StandUp,    // 起立模式
    RL_Control, // 强化学习控制模式
    Disable,    // 失能模式
    Passive,    // 阻尼模式
    Getdown     // 蹲下模式
};

class data_stream
{
private:
    ros::NodeHandle _nh;
    JoyInterface _joy_stick;

    std::thread _data_stream_thread;
    std::vector<float> command_scale = {1.5, 1.5, 1.5};
    thread_Timer _data_stream_timer;

    void _data_streaming_callback();

public:
    data_stream(const std::string &sim_engine);
    ~data_stream();

    std::vector<float> command;
    hardware_interface *hardware;

    RobotState_List Previous_state;
    RobotState_List Current_state_;
    RobotState_List Next_state;

    void joy_stick_command_update();

    void _data_streaming();

    bool running = false;
};

data_stream::data_stream(const std::string &sim_engine)
{
    running = true;

    _data_stream_timer.start(500.0, std::bind(&data_stream::_data_streaming_callback, this), "data_stream_timer");

    Previous_state = RobotState_List::Disable;
    Current_state_ = RobotState_List::Disable;
    Next_state = RobotState_List::Disable;

    command.resize(3);

    if (sim_engine == "gazebo")
        hardware = new gazebo_interface();
    else if (sim_engine == "mujoco")
        hardware = new mujoco_interface();
}

data_stream::~data_stream()
{
    running = false;
}

void data_stream::_data_streaming_callback()
{
    if (!running)
    {
        return;
    }

    try
    {
        joy_stick_command_update();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception in data streaming callback: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "Unknown exception in data streaming callback" << std::endl;
    }
}

void data_stream::joy_stick_command_update()
{

    command[0] = _joy_stick.current_state_.LS[1] * command_scale[1]; // x
    command[1] = _joy_stick.current_state_.LS[0] * command_scale[0]; // y
    command[2] = _joy_stick.current_state_.RS[0] * command_scale[2]; // yaw

    if (_joy_stick.current_state_.LB && _joy_stick.current_state_.RB || 
        !joint_safety_check(hardware->pos,hardware->vel) || 
        !body_safety_check(hardware->gravity_projection))
    {
        Next_state = RobotState_List::Passive;
        return;
    }

    bool A_pressed = _joy_stick.current_state_.A;
    bool Y_pressed = _joy_stick.current_state_.Y;
    bool X_pressed = _joy_stick.current_state_.X;
    bool B_pressed = _joy_stick.current_state_.B;
    bool RB_pressed = _joy_stick.current_state_.RB;

    switch (Current_state_)
    {
    case RobotState_List::Disable:
        if (Y_pressed && RB_pressed) // Y+RB进入阻尼模式
            Next_state = RobotState_List::Passive;
        break;

    case RobotState_List::Passive:
        if (A_pressed && !Y_pressed && !X_pressed && !B_pressed) // A键进入起立
            Next_state = RobotState_List::StandUp;
        break;

    case RobotState_List::StandUp:
        if (Y_pressed && !A_pressed && !X_pressed && !B_pressed) // Y键进入蹲下
            Next_state = RobotState_List::Getdown;
        else if (X_pressed && !A_pressed && !Y_pressed && !B_pressed) // X键进入RL控制
            Next_state = RobotState_List::RL_Control;
        break;

    case RobotState_List::RL_Control:
        if (A_pressed && !Y_pressed && !X_pressed && !B_pressed) // A键进入起立
            Next_state = RobotState_List::StandUp;
        else if (Y_pressed && !A_pressed && !X_pressed && !B_pressed) // Y键进入蹲下
            Next_state = RobotState_List::Getdown;
        break;

    case RobotState_List::Getdown:
        if (A_pressed && !Y_pressed && !X_pressed && !B_pressed) // A键进入起立
            Next_state = RobotState_List::StandUp;
        break;
    }
}
