#ifndef FSM_STATE_HPP
#define FSM_STATE_HPP

#include <iostream>
#include <vector>
#include <robot_msgs/RobotState.h>   //机器人状态消息
#include <robot_msgs/RobotCommand.h> //机器人控制命令消息
#include <deploy/data_update.hpp>

enum class Running_State // 状态机的运行状态
{
    Begin,
    Running,
    Over
};

class FSM_State
{
public:
    FSM_State(const std::string &state_name) : state_name_(state_name) {}; // 构造函数，用于初始化状态机的名称
                                                                           // 状态机的名称

    virtual void enter() = 0; // 状态机的进入函数，在状态机切换到该状态时调用
    virtual void run() = 0;                                                                                                                 // 状态机的更新函数，在每个时间步调用
    virtual void exit() = 0;                                                                                                                // 状态机的退出函数，在状态机从该状态切换到其他状态时调用

    virtual ~FSM_State() = default;

    const std::string &get_state_name() const
    { // 获取状态机的名称
        return state_name_;
    }

    std::string state_name_;

    Running_State running_state = Running_State::Begin; // 状态机的运行状态
    robot_msgs::RobotState *robot_state_=nullptr; // 机器人状态
    robot_msgs::RobotCommand *robot_command_=nullptr; // 机器人控制命令

    RobotState_List *previous_state_=nullptr; // 上一个状态机
};

#endif
