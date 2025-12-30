#include "interface_torchscript.hpp"

InterfaceTorchscript::InterfaceTorchscript(const std::string &model_path)
{
    try
    {
        // model = torch::jit::load(model_path, torch::Device(torch::kCPU));
        model = torch::jit::load(model_path);
        std::cout << "Successfully loaded Torch model" << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cout << "Failed to load Torch model " << std::endl;
    }
}

InterfaceTorchscript::~InterfaceTorchscript() noexcept
{
}

void InterfaceTorchscript::push_forward()
{
    try
    {
        // Convert observation vector to Torch tensor
        std::vector<float> current_obs = normalize_obs();
        // Clip observation
        data_clip(current_obs);

        auto input_tensor = torch::tensor(current_obs, torch::kFloat32).reshape({1, static_cast<int64_t>(current_obs.size())});

        // Disable gradient computation before each forward pass
        torch::autograd::GradMode::set_enabled(false);

        // Ensure single-threaded execution (critical for performance!)
        torch::set_num_threads(1);

        // Execute forward inference
        auto output = model.forward({input_tensor}).toTensor();

        // Convert output tensor to vector
        auto output_accessor = output.accessor<float, 2>();
        std::vector<float> act;
        for (int i = 0; i < output_accessor.size(1); ++i)
        {
            act.push_back(output_accessor[0][i]);
        }

        data_clip(act);
        // Store action in data
        for (int i = 0; i < act.size() && i < data.act.action.size(); ++i)
        {
            data.act.action[i] = act[i] * action_scale_robot[i];
            data.act.action[i] += dof_pos_rl[i];
        }

        data.obs.last_action = act;
    }
    catch (const std::exception &e) // 捕获Torch推理过程中可能抛出的异常
    {
        std::cout << "Torch inference error: " << e.what() << std::endl;
    }
}

void InterfaceTorchscript::enter()
{
    run_cnt_ = 0;
}

std::vector<float> InterfaceTorchscript::get_act() // 获取策略网络的输出（关节的绝对值参数）
{
    return data.act.action;
}

std::vector<float> InterfaceTorchscript::normalize_obs()
{

    // 进行四元数的重新映射为相应的重力的投影
    std::vector<float> projected_gravity = computeGravityProjection(data.obs.gravity_vec);
    data.obs.gravity_vec_ = projected_gravity;

    // 创建一个临时变量来存储处理后的数据，不修改原始data
    std::vector<float> scaled_angular_vel(3);
    for (size_t i = 0; i < 3; i++)
        scaled_angular_vel[i] = data.obs.angular_vel[i] * omega_scale_; // 角速度缩放

    // 创建临时变量存储处理后的命令
    std::vector<float> scaled_command = data.obs.command;

    // 创建临时变量存储处理后的关节角度
    std::vector<float> adjusted_joint_angle(12);
    for (int i = 0; i < 12; ++i)
        adjusted_joint_angle[i] = data.obs.joint_angle[i] - dof_pos_rl[i]; // 关节角度默认值(全部为当前的默认角度的偏移量)

    // 创建临时变量存储处理后的关节速度
    std::vector<float> adjusted_joint_vel(12);
    for (int i = 0; i < 12; ++i)
        adjusted_joint_vel[i] = data.obs.joint_vel[i] * dof_vel_scale_; // 关节速度默认值(全部为当前的默认速度的偏移量)

    /*==============================================观测量进行整合===================================================*/
    std::vector<float> integrated_obs(obs_dim_);
    int idx = 0;

    // 复制缩放后的角速度 (3维)
    for (int i = 0; i < 3; ++i)
        integrated_obs[idx++] = scaled_angular_vel[i];

    // 复制重力向量 (3维)
    for (int i = 0; i < 3; ++i)
        integrated_obs[idx++] = projected_gravity[i];

    // 复制缩放后的控制指令 (3维)
    for (int i = 0; i < 3; ++i)
        integrated_obs[idx++] = scaled_command[i];

    // 复制调整后的关节角度 (12维)
    for (int i = 0; i < 12; ++i)
        integrated_obs[idx++] = adjusted_joint_angle[i];

    // 复制关节速度 (12维)
    for (int i = 0; i < 12; ++i)
        integrated_obs[idx++] = adjusted_joint_vel[i];

    // 复制上一个动作 (12维)
    for (int i = 0; i < 12; ++i)
        integrated_obs[idx++] = data.obs.last_action[i];

    return integrated_obs;
}

std::vector<float> InterfaceTorchscript::computeGravityProjection(const std::vector<float> &quaternion)
{
    std::vector<float> v = {0.0f, 0.0f, -1.0f}; // 世界坐标系下的重力向量
    std::vector<float> projected_gravity(3, 0.0f);

    float q_w = quaternion[0];
    float q_x = quaternion[1];
    float q_y = quaternion[2];
    float q_z = quaternion[3];

    float v_x = v[0];
    float v_y = v[1];
    float v_z = v[2];

    float a_x = v_x * (2.0f * q_w * q_w - 1.0f);
    float a_y = v_y * (2.0f * q_w * q_w - 1.0f);
    float a_z = v_z * (2.0f * q_w * q_w - 1.0f);

    float cross_x = q_y * v_z - q_z * v_y;
    float cross_y = q_z * v_x - q_x * v_z;
    float cross_z = q_x * v_y - q_y * v_x;

    float b_x = cross_x * q_w * 2.0f;
    float b_y = cross_y * q_w * 2.0f;
    float b_z = cross_z * q_w * 2.0f;

    float dot = q_x * v_x + q_y * v_y + q_z * v_z;

    float c_x = q_x * dot * 2.0f;
    float c_y = q_y * dot * 2.0f;
    float c_z = q_z * dot * 2.0f;

    projected_gravity[0] = a_x - b_x + c_x;
    projected_gravity[1] = a_y - b_y + c_y;
    projected_gravity[2] = a_z - b_z + c_z;

    return projected_gravity;
}

void InterfaceTorchscript::denormalize_act()
{
    for (size_t i = 0; i < 12; i++)
    {
        data.act.action[i] *= action_scale_robot[i];
        data.act.action[i] += dof_pos_rl[i];
    }
    // return data.act.action;
}

void InterfaceTorchscript::set_obs(robot_msgs::RobotState robot_state, std::vector<float> command)
{
    // 更新角速度数据 (3维)
    data.obs.angular_vel[0] = robot_state.imu.gyroscope[0];
    data.obs.angular_vel[1] = robot_state.imu.gyroscope[1];
    data.obs.angular_vel[2] = robot_state.imu.gyroscope[2];

    // 更新四元数数据 (4维)(在实际推理的过程中会转化为重力向量在机器人坐标系下的投影，归一化为三维)
    data.obs.gravity_vec[0] = robot_state.imu.quaternion[0]; // w
    data.obs.gravity_vec[1] = robot_state.imu.quaternion[1]; // x
    data.obs.gravity_vec[2] = robot_state.imu.quaternion[2]; // y
    data.obs.gravity_vec[3] = robot_state.imu.quaternion[3]; // z

    // 更新控制指令 (3维) - 这里使用默认值，实际应用中应该从外部获取
    data.obs.command[0] = command[0]; // x方向速度
    data.obs.command[1] = command[1]; // y方向速度
    data.obs.command[2] = command[2]; // 旋转速度

    // 更新关节角度 (12维)
    for (int i = 0; i < 12; ++i)
    {
        data.obs.joint_angle[i] = robot_state.motor_state[i].q;
    }

    // 更新关节速度 (12维)
    for (int i = 0; i < 12; ++i)
    {
        data.obs.joint_vel[i] = robot_state.motor_state[i].dq;
    }

    // 上一时刻的动作 (12维)
    // 在模型推理的过程中就已经更新了上一时刻的动作，每一次推理就会更新一次上一时刻的动作
}
