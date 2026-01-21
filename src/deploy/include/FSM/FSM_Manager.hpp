#pragma once

#include <chrono>
#include <thread>
#include <vector>
#include <iostream>
#include <unistd.h>
#include "FSM_State.hpp"
#include <robot_msgs/RobotState.h>
#include "FSM_State_Collection.hpp"
#include <robot_msgs/RobotCommand.h>
#include "data_stream/data_stream.hpp"

class FSM_Manager
{
private:
    struct FSM_List
    {
        FSM_State *Disable = nullptr;
        FSM_State *Passive = nullptr;
        FSM_State *StandUp = nullptr;
        FSM_State *Getdown = nullptr;
        FSM_State *RL_Control = nullptr;
    } FSM_List;

    FSM_State *current_state_ = nullptr;

    std::string _model_type;
    std::string _sim_engine;

    data_stream Data;

public:
    FSM_Manager(const std::string &sim_engine, const std::string &model_type);
    ~FSM_Manager();

    void run();

    FSM_State *state_change(RobotState_List &next_state);
};
FSM_Manager::FSM_Manager(const std::string &sim_engine, const std::string &model_type) : Data(sim_engine), _model_type(model_type), _sim_engine(sim_engine) // 创建状态机列表
{
    FSM_List.Disable = new FSM_Disable("Disable", Data.hardware);
    FSM_List.Passive = new FSM_Passive("Passive", Data.hardware);
    FSM_List.StandUp = new FSM_Getup("StandUp", Data.hardware, &Data.Previous_state);
    FSM_List.Getdown = new FSM_Getdown("Getdown", Data.hardware);
    FSM_List.RL_Control = new FSM_Rl_control("RL_Control", Data.hardware, _model_type, &Data.command);
    current_state_ = FSM_List.Disable;

    current_state_->enter();
}

FSM_Manager::~FSM_Manager()
{
    delete FSM_List.Disable;
    delete FSM_List.Passive;
    delete FSM_List.StandUp;
    delete FSM_List.Getdown;
    delete FSM_List.RL_Control;
}

FSM_State *FSM_Manager::state_change(RobotState_List &next_state)
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

void FSM_Manager::run()
{
        if (current_state_ == nullptr)
        {
            std::cerr << "错误：当前状态机指针为空！" << std::endl;
            return;
        }

        if (Data.Next_state != Data.Current_state)
        {
            current_state_->exit();
            Data.Previous_state = Data.Current_state;
            Data.Current_state = Data.Next_state;
            current_state_ = state_change(Data.Next_state);

            current_state_->enter();
        }
        else
        {
            current_state_->run();
        }
}