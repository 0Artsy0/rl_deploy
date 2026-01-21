#pragma once
/**
 * @brief 机器人参数类
 *
 */

#include <ros/ros.h> // ros头文件
#include <string>    // 字符串库
#include <iostream>  // 标准输入输出流
#include <vector>    // 向量库

class robot_param
{
public:
    const std::string robot_name = "go2"; // 机器人名称

    /*====================================================base_parameters============================================================= */
    const float Safety_Factor = 0.8; // 安全系数因子
    const int dof_nums = 12;

    // 关节额定参数
    const float Max_torque = 30.0 * Safety_Factor; // 电机最大扭矩(N·m)
    const float Max_speed = 14.0 * Safety_Factor;  // 电机最大速度(rad/s)

    std::vector<std::string> joint_name = // 关节名称
        {
            "FR_hip", "FR_thigh", "FR_calf",
            "FL_hip", "FL_thigh", "FL_calf",
            "RR_hip", "RR_thigh", "RR_calf",
            "RL_hip", "RL_thigh", "RL_calf"};

    std::vector<std::vector<float>> joint_limit = // 关节参数
        {
            {-1.06, 1.06}, // Hip
            {-1.57, 3.49},  // Thigh
            {-2.74, -0.84}   // Calf
    };

   std::vector<float> _dof_pos_robot = {
        0.00, 0.80, -1.50,
        0.00, 0.80, -1.50,
        0.00, 0.80, -1.50,
        0.00, 0.80, -1.50};

    std::vector<float> kp_fix = {
        60.0, 80.0, 80.0,
        60.0, 80.0, 80.0,
        60.0, 80.0, 80.0,
        60.0, 80.0, 80.0};

    std::vector<float> kd_fix = {
        3.0, 2.0, 2.0,
        3.0, 2.0, 2.0,
        3.0, 2.0, 2.0,
        3.0, 2.0, 2.0};

    /*====================================================rl_parameters============================================================= */

    float dt = 0.005;   // 时间步长
    int decimation = 4; // 动作执行间隔
    const int _obs_dim = 9 + 3 * dof_nums;
    const int _act_dim = dof_nums;

    float _omega_scale = 0.25;   // 角速度缩放因子
    float _dof_pos_scale = 1;    // 关节位置缩放因子
    float _dof_vel_scale = 0.05; // 关节速度缩放因子
    float _command_scale = 2.0;  // 控制命令缩放因子

    std::vector<float> _dof_pos_rl = {
        0.00, 0.80, -1.50,
        0.00, 0.80, -1.50,
        0.00, 0.80, -1.50,
        0.00, 0.80, -1.50};

    std::vector<float> kp_rl = {
        60.0, 80.0, 80.0,
        60.0, 80.0, 80.0,
        60.0, 80.0, 80.0,
        60.0, 80.0, 80.0};
        
    std::vector<float> kd_rl = {
        3.0, 2.0, 2.0,
        3.0, 2.0, 2.0,
        3.0, 2.0, 2.0,
        3.0, 2.0, 2.0};


    std::vector<float> _action_scale_robot = {0.125, 0.25, 0.25,
                                              0.125, 0.25, 0.25,
                                              0.125, 0.25, 0.25,
                                              0.125, 0.25, 0.25};

    std::vector<std::string> observation = {
        "angular_vel",        // 3
        "gravity_projection", // 3
        "command",            // 3
        "joint_angle",        // 12
        "joint_vel",          // 12
        "last_action"         // 12
    };
};
