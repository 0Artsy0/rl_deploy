#ifndef FSM_STATE_HPP
#define FSM_STATE_HPP

#include <iostream>
#include <vector>
#include <robot_msgs/RobotState.h>   //机器人状态消息
#include <robot_msgs/RobotCommand.h> //机器人控制命令消息
#include "data_stream/data_stream.hpp"
#include "robot_param.hpp"
#include "hardware_interface/hardware_interface.hpp"

class FSM_State
{
public:
    robot_param _robot_param;

    std::vector<std::vector<float>> _cmd;

    FSM_State(const std::string &state_name, hardware_interface *interface) : _state_name(state_name), _interface(interface) 
    {
        _cmd={
            std::vector<float>( _robot_param.dof_nums, 0.0f ), // 位置指令
            std::vector<float>( _robot_param.dof_nums, 0.0f ), // 速度指令
            std::vector<float>( _robot_param.dof_nums, 0.0f ), // 力矩指令
            std::vector<float>( _robot_param.dof_nums, 0.0f ), // kp指令
            std::vector<float>( _robot_param.dof_nums, 0.0f ), // kd指令
        };
    }; 
    virtual void enter() = 0;
    virtual void run() = 0;
    virtual void exit() = 0;

    virtual ~FSM_State() = default;

    const std::string &get_state_name() const
    { // 获取状态机的名称
        return _state_name;
    }

    std::string _state_name;
    
    hardware_interface *_interface=nullptr; // 硬件接口
    RobotState_List *_previous_state=nullptr; // 上一个状态机

};

#endif
