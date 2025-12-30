#pragma once

#include "FSM_State.hpp"
#include <robot_msgs/RobotState.h>                   //机器人状态消息
#include <robot_msgs/RobotCommand.h>                 //机器人控制命令消息
#include "PPO/onnx/interface_onnx.hpp"               // 修改：使用双引号包含项目内头文件
#include "PPO/Torchscript/interface_torchscript.hpp" // 修改：使用双引号包含项目内头文件
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <vector>
#include "deploy/data_update.hpp"

#define YELLOW "\033[33m"
#define RED "\033[31m"
#define GREEN "\033[32m"
#define BLUE "\033[34m"
#define WHITE "\033[0m"
#define RESET "\033[0m"

const float Hip = 0.2;
const float Thigh = 1.375;
const float Calf = -2.75;

std::vector<float> init_pos =
    {0.0, 0.8, -1.5,
     0.0, 0.8, -1.5,
     0.0, 0.8, -1.5,
     0.0, 0.8, -1.5};

std::vector<float> kp_fix =
    {80.0, 80.0, 80.0,
     80.0, 80.0, 80.0,
     80.0, 80.0, 80.0,
     80.0, 80.0, 80.0};

std::vector<float> kd_fix = std::vector<float>(12, 3.0f);

std::vector<float> kp_rl =
    {80.0, 80.0, 80.0,
     80.0, 80.0, 80.0,
     80.0, 80.0, 80.0,
     80.0, 80.0, 80.0};

std::vector<float> kd_rl = std::vector<float>(12, 3.0f);

/*============================================================失能状态机======================================================================*/
class FSM_Disable : public FSM_State // 失能状态状态机
{
private:
public:
    FSM_Disable(const std::string &state_name, robot_msgs::RobotState *robot_state, robot_msgs::RobotCommand *robot_command) : FSM_State(state_name)
    {
        robot_state_ = robot_state;
        robot_command_ = robot_command;
    }; // 构造函数，用于初始化状态机的名称
    ~FSM_Disable() = default; // 析构函数

    void enter() override
    {
        // 机器人的状态更新函数的接口需要重新编写
        std::cout << YELLOW << "\nEnter disable state" << RESET << std::endl;
        running_state = Running_State::Begin;
    }
    void run() override
    {
        if (running_state == Running_State::Begin)
        {
            running_state = Running_State::Running;
        }
        for (auto &vel : robot_command_->motor_command)
        {
            vel.dq = 0.0f;
            vel.q = 0.0f;
            vel.kp = 0.0f;
            vel.kd = 0.0f;
            vel.tau = 0.0f;
        }
        if (running_state == Running_State::Running)
        {
            running_state = Running_State::Over;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    void exit() override // 状态机的退出函数，在状态机从该状态切换到其他状态时调用
    {
        std::cout << GREEN << "Exit disable state" << RESET << std::endl;
    }
};

/*============================================================阻尼状态机======================================================================*/
class FSM_Passive : public FSM_State // 阻尼状态状态机
{
private:
public:
    FSM_Passive(const std::string &state_name, robot_msgs::RobotState *robot_state, robot_msgs::RobotCommand *robot_command) : FSM_State(state_name)
    {
        robot_state_ = robot_state;
        robot_command_ = robot_command;
    }; // 构造函数，用于初始化状态机的名称
    ~FSM_Passive() = default; // 析构函数

    void enter() override
    {
        std::cout << YELLOW << "\nEnter passive state" << RESET << std::endl;
        running_state = Running_State::Begin;
    }
    void run() override
    {
        if (running_state == Running_State::Begin)
        {
            running_state = Running_State::Running;
        }
        for (auto &vel : robot_command_->motor_command)
        {
            vel.q = 0.0f;
            vel.dq = 0.0f;
            vel.kp = 0.0f;
            vel.kd = 5.0f;
            vel.tau = 0.0f;
        }
        if (running_state == Running_State::Running)
        {
            running_state = Running_State::Over;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    void exit() override // 状态机的退出函数，在状态机从该状态切换到其他状态时调用
    {
        std::cout << GREEN << "Exit passive state" << RESET << std::endl;
    }
};
/*============================================================站起状态机======================================================================*/
class FSM_Getup : public FSM_State // 站起状态状态机
{
private:
    const int during = 800;
    float percent = 0;
    int count = 0;

    int version = 0; // 站起来的版本
    int flag = 0;    // 分阶段进行起立

    RobotState_List previous_state;     // 记录上一时刻的状态机的状态
    robot_msgs::RobotState Init_state_; // 记录初始状态

    std::vector<float> pre_stand_pos = {0.0, Thigh, Calf,
                                        0.0, Thigh, Calf,
                                        0.0, Thigh, Calf,
                                        0.0, Thigh, Calf}; // 记录初始位置

public:
    FSM_Getup(const std::string &state_name, robot_msgs::RobotState *robot_state, robot_msgs::RobotCommand *robot_command, RobotState_List *previous_state) : FSM_State(state_name)
    {
        robot_state_ = robot_state;
        robot_command_ = robot_command;
        previous_state_ = previous_state;
    }; // 构造函数，用于初始化状态机的名称
    ~FSM_Getup() = default; // 析构函数

    void enter() override
    {
        // 机器人的状态更新函数的接口需要重新编写
        std::cout << YELLOW << "\nEnter getup state" << RESET << std::endl;
        Init_state_ = *robot_state_; // 记录初始状态

        count = 0;
        percent = 0; // 差分进程进行初始化
        flag = 0;    // 分阶段进行起立

        // 根据根据上一时刻的状态机的版本，来判断站起来的版本
        // 添加指针安全检查
        if (previous_state_ == nullptr)
        {
            std::cout << RED << "Error: previous_state_ is null!" << RESET << std::endl;
            version = 1;
        }
        else
        {
            if (*previous_state_ == RobotState_List::RL_Control)
            {
                version = 1;
            }
            else if (*previous_state_ == RobotState_List::Passive || *previous_state_ == RobotState_List::Getdown) // 如果是阻尼模式或者是蹲下模式
            {
                version = 2;
            }
        }

        running_state = Running_State::Begin;
    }

    void run() override
    {
        if (running_state == Running_State::Begin)
        {
            running_state = Running_State::Running; // 开始进行running
        }

        if ((version == 1 && flag == 1) || (version == 2 && flag == 2))
            running_state = Running_State::Over; // 整体的起立过程结束

        if ((count == during && flag <= 1 && version == 2) || (count == during && flag == 0 && version == 1))
        {
            count = 0;
            flag++;
        }

        percent = static_cast<float>(count) / static_cast<float>(during);

        if (version == 1 && flag == 0)
        {
            for (int i = 0; i < 12; i++)
            {
                robot_command_->motor_command[i].q = Init_state_.motor_state[i].q * (1 - percent) + init_pos[i] * percent;
                robot_command_->motor_command[i].kp = kp_fix[i];
                robot_command_->motor_command[i].kd = kd_fix[i];
            }
        }
        else if (version == 2)
        {
            switch (flag)
            {
            case 0: // pre _stand
                for (int i = 0; i < 12; i++)
                {
                    robot_command_->motor_command[i].q = Init_state_.motor_state[i].q * (1 - percent) + pre_stand_pos[i] * percent;
                    robot_command_->motor_command[i].kp = kp_fix[i];
                    robot_command_->motor_command[i].kd = kd_fix[i];
                }
                break;
            case 1: // stand
                for (int i = 0; i < 12; i++)
                {
                    robot_command_->motor_command[i].q = pre_stand_pos[i] * (1 - percent) + init_pos[i] * percent;
                    robot_command_->motor_command[i].kp = kp_fix[i];
                    robot_command_->motor_command[i].kd = kd_fix[i];
                }
                break;
            default:
                break;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));

        if (count <= 1000)
            count++;
    }

    void exit() override // 状态机的退出函数，在状态机从该状态切换到其他状态时调用
    {
        std::cout << GREEN << "Exit getup state" << RESET << std::endl;
    }
};
/*============================================================蹲下状态机======================================================================*/
class FSM_Getdown : public FSM_State // 蹲下状态状态机
{
private:
    const int during = 800;
    int count = 0;
    float percent = 0;
    int flag = 0; // 分阶段进行蹲下

    int stage_flag = 0;

    robot_msgs::RobotState Init_state_; // 记录初始状态

    std::vector<float> final_getdown_pos = {0.0, Thigh, Calf,
                                            0.0, Thigh, Calf,
                                            0.0, Thigh, Calf,
                                            0.0, Thigh, Calf}; // 记录初始位置

public:
    FSM_Getdown(const std::string &state_name, robot_msgs::RobotState *robot_state, robot_msgs::RobotCommand *robot_command) : FSM_State(state_name)
    {
        robot_state_ = robot_state;
        robot_command_ = robot_command;
    }; // 构造函数，用于初始化状态机的名称
    ~FSM_Getdown() = default; // 析构函数

    void enter() override
    {
        std::cout << YELLOW << "\nEnter getdown state" << RESET << std::endl;
        Init_state_ = *robot_state_; // 记录初始状态

        count = 0;
        percent = 0;    // 差分进程进行初始化
        flag = 0;       // 分阶段进行蹲下
        stage_flag = 0; // 每次进入蹲下状态机，都需要将阶段标志位重置为0
        running_state = Running_State::Begin;
    }
    void run() override
    {
        if (running_state == Running_State::Begin)
        {
            running_state = Running_State::Running;
        }

        if (flag == 3)
        {
            running_state = Running_State::Over;
        }

        if (count == during && flag <= 2)
        {
            count = 0;
            flag++; // 分阶段进行蹲下
        }

        percent = static_cast<float>(count) / static_cast<float>(during);

        switch (flag)
        {
        case 0: // 第一阶段：从任何状态回复到站立位置
            for (int i = 0; i < 12; i++)
            {
                robot_command_->motor_command[i].q = Init_state_.motor_state[i].q * (1 - percent) + init_pos[i] * percent;
                robot_command_->motor_command[i].kp = kp_fix[i];
                robot_command_->motor_command[i].kd = kd_fix[i];
            }
            break;
        case 1: // 蹲到最终的位置
            for (int i = 0; i < 12; i++)
            {
                robot_command_->motor_command[i].q = init_pos[i] * (1 - percent) + final_getdown_pos[i] * percent;
                robot_command_->motor_command[i].kp = kp_fix[i];
                robot_command_->motor_command[i].kd = kd_fix[i];
            }
            break;
        case 2: // 阻尼模式
            for (int i = 0; i < 12; i++)
            {
                robot_command_->motor_command[i].q = 0.0;
                robot_command_->motor_command[i].kp = 0.0;
                robot_command_->motor_command[i].kd = 5.0f;
            }
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));

        if (count <= 1000)
            count++;
    }
    void exit() override // 状态机的退出函数，在状态机从该状态切换到其他状态时调用
    {
        std::cout << GREEN << "Exit getdown state" << RESET << std::endl;
    }
};
/*============================================================强化学习运动控制状态机======================================================================*/
// 目前可能发现的问题，就是关节顺序有点不同，左右腿的关节顺序不同，导致模型训练时的输入输出顺序不同，可能需要进行调整
// 但是我觉得这个问题对整体的影响并不是很大，因为左右腿的关节顺序不同，只是导致了模型训练时的输入输出顺序不同，不会影响模型的训练和推理

class FSM_Rl_control : public FSM_State // 强化学习控制状态机
{
private:
    std::vector<float> *command = nullptr;

    std::string model_path;        // 强化学习模型路径
    std::string putforward_model_; // 前馈模型

    RLDataOperation *model_operation = nullptr; // 强化学习数据操作

    InterfaceTorchscript torchscript_model; // 强化学习torchscript模型
    InterfaceOnnx onnx_model;               // 强化学习onnx模型

    std::string putforward_model; // 强化学习模型路径

    std::vector<float> act; // 强化学习的输出动作
    int run_cnt = 0;

public:
    FSM_Rl_control(const std::string &state_name, robot_msgs::RobotState *robot_state, robot_msgs::RobotCommand *robot_command, const std::string &putforward_model, std::vector<float> *command_)
        : FSM_State(state_name),
          putforward_model_(putforward_model),
          model_path("/home/lym/zhang/deploy/src/policy/go2"),
          torchscript_model(model_path + "/policy.pt"),
          onnx_model(model_path + "/policy.onnx"),
          command(command_)
    {
        robot_state_ = robot_state;
        robot_command_ = robot_command;

        if (putforward_model_ == "None")
        {
            std::cout << "No putforward model" << std::endl;
        }
        else if (putforward_model_ == "onnx")
        {
            model_operation = &onnx_model;
        }
        else if (putforward_model_ == "torchscript")
        {
            model_operation = &torchscript_model;
        }

        model_operation->dt_ = 0.005;     // 时间步长
        model_operation->decimation_ = 4; // 控制降采样
    };

    void enter() override
    {
        std::cout << YELLOW << "\nEnter Rl_control state" << RESET << std::endl;
        running_state = Running_State::Begin;
        run_cnt = 0;
    }
    void run() override
    {

        running_state = Running_State::Over;

        model_operation->set_obs(*robot_state_, *command); // 观测量的更新

        if (run_cnt % model_operation->decimation_ == 0)
        {
            model_operation->push_forward();  // 模型的推理
            act = model_operation->get_act(); // 获取动作值

            for (int i = 0; i < 12; i++)
            {
                robot_command_->motor_command[i].q = act[i];
                robot_command_->motor_command[i].dq = 0;
                robot_command_->motor_command[i].kp = kp_rl[i];
                robot_command_->motor_command[i].kd = kd_rl[i];
                robot_command_->motor_command[i].tau = 0;
            }
        }

        run_cnt++;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    void exit() override // 状态机的退出函数，在状态机从该状态切换到其他状态时调用
    {
        std::cout << GREEN << "Exit Rl_control state" << RESET << std::endl;
    }
};