#ifndef RL_DATA_OPERATATION_HPP
#define RL_DATA_OPERATATION_HPP

#include <iostream>
#include <vector>
#include <stdexcept>
#include <numeric>
#include "robot_msgs/RobotState.h"
#include "robot_msgs/RobotCommand.h"

struct _observation // 观测量，一共45维度
{
    std::vector<float> angular_vel;  // 角速度（3维度）
    std::vector<float> gravity_vec;  // 重力向量（实际上需要的是四元数，但是我实际上传入的四元数）
    std::vector<float> gravity_vec_; // 重力向量
    std::vector<float> command;      // 控制指令(3维度)
    std::vector<float> joint_angle;  // 当前关节角度(12维度)
    std::vector<float> joint_vel;    // 当前关节速度（12维度）
    std::vector<float> last_action;  // 上一个action（12维度）（纯位置控制）
};

struct _action // 动作量，一共12维度
{
    std::vector<float> action;
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
    int decimation_;
    int run_cnt_;
    float dt_;

    _rl_data data;

    virtual void enter() = 0;

    // 观测相关操作
    virtual void set_obs(robot_msgs::RobotState robot_state, std::vector<float> command) = 0;

    // 动作相关操作
    virtual std::vector<float> get_act() = 0;

    // 动作推理
    virtual void push_forward() = 0;

    // 数据预处理
    virtual std::vector<float> normalize_obs() = 0;
    virtual void denormalize_act() = 0;

    RLDataOperation();
    virtual ~RLDataOperation();

    void data_clip(std::vector<float> &obs); // 观测量裁剪
};

RLDataOperation::RLDataOperation()
{
    // action
    data.act.action.resize(12, 0.0);
    // observation
    data.obs.angular_vel.resize(3, 0.0);
    data.obs.gravity_vec.resize(4, 0.0);  // 目前传入的是四元数
    data.obs.gravity_vec_.resize(3, 0.0); // 重力向量
    data.obs.command.resize(3, 0.0);
    data.obs.joint_angle.resize(12, 0.0);
    data.obs.joint_vel.resize(12, 0.0);
    data.obs.last_action.resize(12, 0.0);
}

RLDataOperation::~RLDataOperation() = default;

void RLDataOperation::data_clip(std::vector<float> &obs)
{
      for (auto &value : obs)
    {
        // // 乘以100，四舍五入，再除以100，保留两位小数
        // value = std::round(value * 100.0f) / 100.0f;

        value = static_cast<int>(value * 100.0f);

        value = static_cast<float>(value);
        value = value / 100.0f;
    }

}

#endif