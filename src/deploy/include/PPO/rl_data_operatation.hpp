#pragma once

#include <iostream>
#include <vector>
#include <stdexcept>
#include <numeric>
#include "robot_msgs/RobotState.h"
#include "robot_msgs/RobotCommand.h"
#include "../robot_param.hpp"

struct _observation // 观测量，一共45维度
{
    std::vector<float> angular_vel;  // 角速度（3维度）
    std::vector<float> gravity_projection; // 重力投影（3维度）
    std::vector<float> command;      // 控制指令(3维度)
    std::vector<float> joint_angle;  // 当前关节角度(12维度)
    std::vector<float> joint_vel;    // 当前关节速度（12维度）
    std::vector<float> last_action;  // 上一个action（12维度）（纯位置控制）
};

struct _action // 动作量，一共12维度
{
    std::vector<std::vector<float>> action;
};

struct _rl_data
{
    std::string obs_group = "obs";     // 观测组
    std::string act_group = "actions"; // 动作组

    _observation obs;
    _action act;
};

class RLDataOperation
{
public:
    RLDataOperation():_robot_param()
    {   
        // observation
        data.obs.angular_vel.resize(3, 0.0);
        data.obs.gravity_projection.resize(3, 0.0); // 重力投影（3维度）
        data.obs.command.resize(3, 0.0);
        data.obs.joint_angle.resize(_robot_param.dof_nums, 0.0);
        data.obs.joint_vel.resize(_robot_param.dof_nums, 0.0);
        data.obs.last_action.resize(_robot_param.dof_nums, 0.0);
    };
    virtual ~RLDataOperation();


    robot_param _robot_param;
    _rl_data data;

    // 数据的相关操作
    std::vector<float> set_obs();
    void set_act(const std::vector<float> &act);
    // 动作相关操作
    std::vector<float> get_act();

    // 动作推理
    virtual void push_forward() = 0;

    void data_clip(std::vector<float> &obs); // 观测量裁剪

    void observation_set(std::vector<float> angular_vel, std::vector<float> gravity_projection, std::vector<float> command, std::vector<float> joint_angle, std::vector<float> joint_vel);

    std::vector<float> gravity_projection; // 重力投影（3维度）
    std::vector<float> command;      // 控制指令(3维度)
    std::vector<float> joint_angle;  // 当前关节角度(12维度)
    std::vector<float> joint_vel;    // 当前关节速度（12维度）); // 观测量设置
};
RLDataOperation::~RLDataOperation() = default;

void RLDataOperation::data_clip(std::vector<float> &obs)//数据裁剪，全部裁剪为小数点后面两位
{
      for (auto &value : obs)
    {
        value = static_cast<int>(value * 100.0f);

        value = static_cast<float>(value);
        value = value / 100.0f;
    }
}

std::vector<float> RLDataOperation::set_obs()
{
/*==============================================观测量进行整合===================================================*/
    std::vector<float> integrated_obs;

    for (size_t i = 0; i < 3; i++)//角速度缩放整合
        integrated_obs.push_back(data.obs.angular_vel[i] * _robot_param._omega_scale); // 角速度缩放
    for (int i = 0; i < 3; ++i)//重力投影缩放
        integrated_obs.push_back(data.obs.gravity_projection[i]); // 重力投影缩放
    for (int i = 0; i < 3; ++i)//控制命令缩放整合
        integrated_obs.push_back(data.obs.command[i]); // 控制命令缩放
    for (int i = 0; i < _robot_param.dof_nums; ++i)//关节角度整合
        integrated_obs.push_back((data.obs.joint_angle[i] - _robot_param._dof_pos_rl[i]) * _robot_param._dof_pos_scale); // 关节角度默认值(全部为当前的默认角度的偏移量)
    for (int i = 0; i < _robot_param.dof_nums; ++i)//关节速度整合
        integrated_obs.push_back(data.obs.joint_vel[i] * _robot_param._dof_vel_scale); // 关节速度默认值(全部为当前的默认速度的偏移量)
    for (int i = 0; i < _robot_param.dof_nums; ++i)//上一个动作整合
        integrated_obs.push_back(data.obs.last_action[i]); // 上一个动作默认值(全部为当前的默认动作的偏移量)

    data_clip(integrated_obs);
    return integrated_obs;
}
void RLDataOperation::set_act(const std::vector<float> &act)
{ 
    std::vector<float> processed_action;
    for (int i = 0; i < act.size(); ++i)
        processed_action.push_back(act[i] * _robot_param._action_scale_robot[i]+ _robot_param._dof_pos_rl[i]);

    data.act.action.push_back(processed_action);

    const size_t max_history_size = 6;
    if (data.act.action.size() > max_history_size) {
        data.act.action.erase(data.act.action.begin());
    }

}
std::vector<float> RLDataOperation::get_act()
{
    // 安全检查：确保动作队列不为空
    if (data.act.action.empty()) {
        std::cerr << "Warning: Action queue is empty! Returning default action." << std::endl;
        return std::vector<float>(_robot_param.dof_nums, 0.0f);
    }

    return data.act.action.back();
}

void RLDataOperation::observation_set(std::vector<float> angular_vel, std::vector<float> gravity_projection, std::vector<float> command, std::vector<float> joint_angle, std::vector<float> joint_vel)
{
    data.obs.angular_vel = angular_vel;
    data.obs.gravity_projection = gravity_projection;
    data.obs.command = command;
    data.obs.joint_angle = joint_angle;
    data.obs.joint_vel = joint_vel;
}
