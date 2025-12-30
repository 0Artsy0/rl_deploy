#pragma once

#include "FSM_State.hpp"
#include <robot_msgs/RobotState.h>   //机器人状态消息
#include <robot_msgs/RobotCommand.h> //机器人控制命令消息
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <vector>
#include "deploy/data_update.hpp" //数据更新流
#include "FSM_State_Collection.hpp"

class FSM_Manager // 状态机管理器
{
private:
    struct FSM_List // 状态机列表
    {
        FSM_State *Disable = nullptr;
        FSM_State *Passive = nullptr;
        FSM_State *StandUp = nullptr;
        FSM_State *Getdown = nullptr;
        FSM_State *RL_Control = nullptr;
    } FSM_List;

    FSM_State *current_state_ = nullptr; // 当前状态机

    std::string putforward_model_;
    std::string sim_engine_;

    data_update data; // 数据流

public:
    FSM_Manager(const std::string &sim_engine, const std::string &putforward_model);
    ~FSM_Manager();

    void run();

    bool running = false; // 状态机运行标志
    FSM_State *state_change(RobotState_List &next_state);
};
FSM_Manager::FSM_Manager(const std::string &sim_engine, const std::string &putforward_model) : data(sim_engine), putforward_model_(putforward_model), sim_engine_(sim_engine) // 创建状态机列表
{
    // 添加状态机
    FSM_List.Disable = new FSM_Disable("Disable", &data.robot_state, &data.robot_command);
    FSM_List.Passive = new FSM_Passive("Passive", &data.robot_state, &data.robot_command);
    FSM_List.StandUp = new FSM_Getup("StandUp", &data.robot_state, &data.robot_command, &data.previous_state);
    FSM_List.Getdown = new FSM_Getdown("Getdown", &data.robot_state, &data.robot_command);
    FSM_List.RL_Control = new FSM_Rl_control("RL_Control", &data.robot_state, &data.robot_command, putforward_model_, &data.command);
    // 设置初始状态机
    current_state_ = FSM_List.Disable; // 初始状态机为无功能模式

    running = true; // 状态机运行标志
}

FSM_Manager::~FSM_Manager() // 析构函数
{
    // 释放状态机内存
    delete FSM_List.Disable;
    delete FSM_List.Passive;
    delete FSM_List.StandUp;
    delete FSM_List.Getdown;
    delete FSM_List.RL_Control;

    running = false; // 状态机运行标志
}

FSM_State *FSM_Manager::state_change(RobotState_List &next_state) // 状态机映射函数,通过状态机名称映射状态机指针
{
    if (next_state == RobotState_List::Disable)
        return FSM_List.Disable;
    else if (next_state == RobotState_List::Passive)
        return FSM_List.Passive;
    else if (next_state == RobotState_List::StandUp)
        return FSM_List.StandUp;
    else if (next_state == RobotState_List::Getdown)
        return FSM_List.Getdown;
    else if (next_state == RobotState_List::RL_Control)
        return FSM_List.RL_Control;
    else
        return nullptr;
}

void FSM_Manager::run() // 状态机的运行函数
{
    current_state_->enter(); // 使能当前状态机

    while (running && ros::ok()) // 状态机运行循环
    {
        // 添加状态有效性检查
        if (current_state_ == nullptr)
        {
            std::cerr << "错误：当前状态机指针为空！" << std::endl;
            break;
        }

        if ((data.next_state != data.current_state_) && current_state_->running_state == Running_State::Over) // 状态机切换条件
        {
            current_state_->exit();                         // 退出当前状态机
            data.previous_state = data.current_state_;      // 更新上一个状态
            data.current_state_ = data.next_state;          // 更新当前状态
            current_state_ = state_change(data.next_state); // 指针切换到下一个状态机
            // 添加状态有效性检查
            if (current_state_ == nullptr)
            {
                std::cerr << "错误：当前状态机指针为空！" << std::endl;
                break;
            }
            else
            {
                current_state_->enter(); // 使能当前状态机
            }
        }
        else
        {
            current_state_->run(); // 运行当前状态机
        }
    }
}
