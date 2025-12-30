#pragma once
/**
 * @brief 数据更新类，用于更新机器人的状态和命令
 * @param robot_state 机器人的状态消息（直接接受过来，供外部进行使用）
 * @param robot_command 机器人的命令消息(需要外部进行处理，更新)
 * @param joy_stick 手柄控制的命令指令，包括机器人的状态更新
 */
#include <robot_msgs/RobotState.h>                        //机器人状态消息
#include <robot_msgs/RobotCommand.h>                      //机器人命令消息
#include <ros/ros.h>                                      //ros头文件
#include <Joy_interface.hpp> //手柄接口头文件
#include <iostream>                                       // 标准输入输出流
#include <thread>                                         // 线程库
#include <vector>                                         // 向量库

enum class RobotState_List
{
    StandUp,    // 起立模式
    RL_Control, // 强化学习控制模式
    Disable,    // 失能模式
    Passive,    // 阻尼模式
    Getdown     // 蹲下模式
};

enum class Model_Safe
{
    safe,  // 正常状态
    danger // 危险状态
};

class data_update
{
private:
    ros::NodeHandle nh_;               // 节点句柄
    ros::Subscriber sub_robot_state_;  // 机器人状态订阅
    ros::Publisher pub_robot_command_; // 机器人命令发布

    JoyInterface joy_stick; // 手柄接口

    std::thread data_update_thread_;                                         // 数据更新线程
    std::vector<std::string> robot_topic = {"/robot_state", "/robot_command"}; // 机器人话题

    std::vector<float> command_scale = {1.2, 1.2, 0.8}; // 控制指令的缩放因子

    const float roll_threshold = 1.2;  // 横滚角度阈值
    const float pitch_threshold = 1.2; // 俯仰角度阈值

public:
    data_update(const std::string &sim_engine);
    ~data_update();

    std::vector<float> command;             // 机器人的控制指令，包括前后左右的速度和旋转的速度
    robot_msgs::RobotState robot_state;     // 机器人状态
    robot_msgs::RobotCommand robot_command; // 机器人命令

    RobotState_List previous_state; // 上一个状态
    RobotState_List current_state_; // 当前状态
    RobotState_List next_state;     // 下一个状态
    Model_Safe model_safe;          // 模型安全状态

    void robot_state_callback(const robot_msgs::RobotState::ConstPtr &msg);
    void data_update_(); //
    void joy_stick_command_update();
    void state_check(); // 状态检查函数，用于检查当前状态是否安全

    bool running = false;
};

data_update::data_update(const std::string &sim_engine)
{
    running = true; // 运行的标志

    //话题的重映射
    robot_topic[0]="/go2_"+sim_engine+robot_topic[0];
    robot_topic[1]="/go2_"+sim_engine+robot_topic[1];

    // ros话题的配对
    sub_robot_state_ = nh_.subscribe<robot_msgs::RobotState>(robot_topic[0], 10, &data_update::robot_state_callback, this); // 订阅机器人状态
    pub_robot_command_ = nh_.advertise<robot_msgs::RobotCommand>(robot_topic[1], 10);                                       // 发布机器人命令

    // 启动消息更新线程
    data_update_thread_ = std::thread(&data_update::data_update_, this);

    previous_state = RobotState_List::Disable; // 初始化上一个状态为起立模式
    current_state_ = RobotState_List::Disable; // 初始化状态为起立模式
    next_state = RobotState_List::Disable;     // 初始化下一个状态为起立模式

    model_safe = Model_Safe::safe; // 初始化模型安全状态为正常状态
    command.resize(3); // 机器人控制指令的初始化，默认为disable状态

    // 控制命令数据的初始化
    robot_command.motor_command.resize(12); // 机器人控制指令的初始化，默认为disable状态
    for (int i = 0; i < 12; i++)
    {
        robot_command.motor_command[i].q = 0.0;
        robot_command.motor_command[i].dq = 0.0;
        robot_command.motor_command[i].kp = 0.0;
        robot_command.motor_command[i].kd = 0.0;
        robot_command.motor_command[i].tau = 0.0;
    }
}

data_update::~data_update()
{
    running = false;
    if (data_update_thread_.joinable())
    {
        data_update_thread_.join(); // 等待线程结束
    }
    std::cout << "data stream close" << std::endl;
}

void data_update::robot_state_callback(const robot_msgs::RobotState::ConstPtr &msg)
{
    robot_state = *msg; // 更新机器人状态
}

/**
 * @brief 数据更新线程主循环函数
 *
 * 该函数运行在独立线程中，负责：
 * 1. 订阅机器人状态话题并更新机器人状态数据
 * 2. 发布机器人控制命令话题
 * 3. 实时读取并处理手柄按键状态
 * 4. 根据手柄输入更新机器人运动控制指令
 *
 * @note 运行频率为500Hz，确保实时性
 * @warning 线程安全需确保，避免数据竞争
 */

void data_update::data_update_()
{
    ros::Rate rate(500); // 500Hz
    while (running)
    {
        joy_stick_command_update();                // 更新手柄命令
        pub_robot_command_.publish(robot_command); // 发布机器人命令
        state_check();                             // 检查当前状态是否安全
        ros::spinOnce();                           // 处理回调
    }
}

void data_update::joy_stick_command_update()
{
    // 控制命令的更新
    command[0] = joy_stick.current_state_.LS[1] * command_scale[1]; // 前后的速度
    command[1] = joy_stick.current_state_.LS[0] * command_scale[0]; // 左右的速度
    command[2] = joy_stick.current_state_.RS[0] * command_scale[2]; // 旋转速度

    // 安全状态下的模式转换
    switch (current_state_)
    {
    case RobotState_List::Disable:
        // 同时按下Y和RB键进入阻尼模式（安全启动）
        if (joy_stick.current_state_.Y && joy_stick.current_state_.RB)
        {
            next_state = RobotState_List::Passive;
        }
        break;

    case RobotState_List::Passive:
        // 按下A键进入起立模式
        if (joy_stick.current_state_.A == 1 && joy_stick.current_state_.Y == 0 && joy_stick.current_state_.X == 0 && joy_stick.current_state_.B == 0)
        {
            next_state = RobotState_List::StandUp;
        }
        break;

    case RobotState_List::StandUp:
        // 只有按下Y键进入蹲下模式
        if (joy_stick.current_state_.Y == 1 && joy_stick.current_state_.A == 0 && joy_stick.current_state_.X == 0 && joy_stick.current_state_.B == 0)
        {
            next_state = RobotState_List::Getdown;
        }
        // 按下X键进入强化学习控制模式
        else if (joy_stick.current_state_.X == 1 && joy_stick.current_state_.A == 0 && joy_stick.current_state_.Y == 0 && joy_stick.current_state_.B == 0)
        {
            next_state = RobotState_List::RL_Control;
        }
        break;

    case RobotState_List::RL_Control:
        // 按下A键进入起立模式
        if (joy_stick.current_state_.A == 1 && joy_stick.current_state_.Y == 0 && joy_stick.current_state_.X == 0 && joy_stick.current_state_.B == 0)
        {
            next_state = RobotState_List::StandUp;
        }
        //按下Y进入到蹲下模式
        if (joy_stick.current_state_.Y == 1 && joy_stick.current_state_.A == 0 && joy_stick.current_state_.X == 0 && joy_stick.current_state_.B == 0)
        {
            next_state = RobotState_List::Getdown;
        }
        break;
    case RobotState_List::Getdown:
        // 按下A键进入起立模式
        if (joy_stick.current_state_.A == 1 && joy_stick.current_state_.Y == 0 && joy_stick.current_state_.X == 0 && joy_stick.current_state_.B == 0)
        {
            next_state = RobotState_List::StandUp;
        }
        break;
        // 这些状态暂时没有特殊处理
        break;
    }

        if ((joy_stick.current_state_.LB && joy_stick.current_state_.RB) || model_safe == Model_Safe::danger)
    {
        next_state = RobotState_List::Passive; // LB+RB紧急停止
        return;
    }
}

void data_update::state_check()
{
    float roll;
    float pitch;

    // 使用robot_state中的imu的四元数计算出横滚角度和俯仰角度
    roll = atan2(2.0 * (robot_state.imu.quaternion[0] * robot_state.imu.quaternion[1] + robot_state.imu.quaternion[2] * robot_state.imu.quaternion[3]),
                 1.0 - 2.0 * (robot_state.imu.quaternion[1] * robot_state.imu.quaternion[1] + robot_state.imu.quaternion[2] * robot_state.imu.quaternion[2]));
    pitch = asin(2.0 * (robot_state.imu.quaternion[0] * robot_state.imu.quaternion[2] - robot_state.imu.quaternion[3] * robot_state.imu.quaternion[1]));

    // 只要有一个角超过阈值，就标记为危险状态
    if (std::abs(roll) > roll_threshold || std::abs(pitch) > pitch_threshold)
    {
        model_safe = Model_Safe::danger; // 标记模型为危险状态
    }
    else
    {
        model_safe = Model_Safe::safe; // 标记模型为安全状态
    }
}
