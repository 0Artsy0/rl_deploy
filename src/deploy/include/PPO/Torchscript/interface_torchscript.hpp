#ifndef INTERFACE_TORCHSCRIPT_HPP_
#define INTERFACE_TORCHSCRIPT_HPP_

#include "../rl_data_operatation.hpp"
#include "iostream"
#include "vector"
#include <torch/script.h>
#include <ATen/Parallel.h>
#include <torch/utils.h>
#include <cmath>

#include <robot_msgs/RobotState.h>

#define max_cmd_vel_ 1.2 // 最大速度
#define max_rot_vel_ 1.2 // 最大旋转速度

class InterfaceTorchscript : public RLDataOperation
{
private:
    std::string model_path_; // 模型的文件路径
    std::string device_;     // 模型推理的硬件平台，包含CPU或者GPU

    torch::jit::script::Module model; // TorchScript模块

    float omega_scale_ = 0.25;   // 角速度缩放因子
    float dof_vel_scale_ = 0.05; // 关节速度缩放因子
    float command_scale = 2.0;   // 关节位置缩放因子

    std::vector<float> action_scale_robot = {0.125, 0.25, 0.25,
                                             0.125, 0.25, 0.25,
                                             0.125, 0.25, 0.25,
                                             0.125, 0.25, 0.25};
    const int obs_dim_ = 45;
    const int act_dim_ = 12;

    std::vector<float> dof_pos_robot = {
        0.00, 0.80, -1.50,
        0.00, 0.80, -1.50,
        0.00, 0.80, -1.50,
        0.00, 0.80, -1.50};
    std::vector<float> dof_pos_rl = {
        0.00, 0.80, -1.50,
        0.00, 0.80, -1.50,
        0.00, 0.80, -1.50,
        0.00, 0.80, -1.50};

public:
    InterfaceTorchscript(const std::string &model_path); // 第一个参数是模型的文件路径，第二个参数是模型推理的硬件平台，包含CPU或者GPU
    ~InterfaceTorchscript() noexcept;

    void set_obs(robot_msgs::RobotState robot_state, std::vector<float> command) override;
    std::vector<float> get_act() override;

    std::vector<float> normalize_obs() override;
    void denormalize_act() override;

    std::vector<float> computeGravityProjection(const std::vector<float> &quaternion);

    void push_forward() override;
    void enter();
};

#endif