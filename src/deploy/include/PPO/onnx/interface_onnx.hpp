#ifndef INTERFACE_ONNX_HPP
#define INTERFACE_ONNX_HPP

#include "../rl_data_operatation.hpp"
#include "iostream"
#include <onnxruntime_cxx_api.h>
#include <vector>
#include <robot_msgs/RobotState.h>

#define max_cmd_vel_ 1.2 // 最大速度
#define max_rot_vel_ 1.2 // 最大旋转速度

class InterfaceOnnx : public RLDataOperation
{
private:
    std::string model_path_; // 模型的文件路径

    Ort::Env env_;                       // onnxruntime环境
    Ort::SessionOptions session_options; // 会话选项
    Ort::MemoryInfo memory_info_;        // onnxruntime内存信息
    Ort::Session session_;               // onnxruntime会话

    std::vector<const char *> input_names_;
    std::vector<const char *> output_names_;

    const int obs_dim_ = 45;
    const int act_dim_ = 12;
    const int motor_num = 12;

    // 模型推理测试参数
    std::vector<float> dof_pos_robot, dof_pos_rl;                     // 机器人的默认关节位置以及策略关节位置
    std::vector<float> kp_, kd_;                                      // 关节位置和速度的比例系数和微分系数
    std::vector<float> cmd_vel, gravity_direction = {0.0, 0.0, -1.0}; // 机器人的默认关节速度以及策略关节速度
    float *action_data;

    float omega_scale_ = 0.25;   // 角速度缩放因子
    float dof_vel_scale_ = 0.05; // 关节速度缩放因子
    float command_scale = 2.0;      // 关节位置缩放因子

    std::vector<float> action_scale_robot = {0.125, 0.25, 0.25,
                                             0.125, 0.25, 0.25,
                                             0.125, 0.25, 0.25,
                                             0.125, 0.25, 0.25};

public:
    InterfaceOnnx(const std::string &model_path); // 第一个参数是模型的文件路径，第二个参数是模型推理的硬件平台，包含CPU或者GPU
    ~InterfaceOnnx();

    void set_obs(robot_msgs::RobotState robot_state, std::vector<float> command) override;
    std::vector<float> get_act() override;

    std::vector<float> normalize_obs() override;
    void denormalize_act() override;

    std::vector<float> computeGravityProjection(const std::vector<float> &quaternion);

    void push_forward() override;
    void enter();
};

#endif