#include "interface_onnx.hpp"

InterfaceOnnx::InterfaceOnnx(const std::string &model_path)
    : model_path_(model_path),
      env_(ORT_LOGGING_LEVEL_WARNING, "policy_forward"),
      session_options(),
      memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)),
      session_(nullptr)
{
    // 设置会话选项
    session_options.SetIntraOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_ = Ort::Session(env_, model_path_.c_str(), session_options);

    input_names_.push_back(data.obs_group.c_str());  // 输入节点的名称
    output_names_.push_back(data.act_group.c_str()); // 输出节点的名称

    dof_pos_robot.resize(12); // 机器人的默认位置
    dof_pos_robot = {0.0000, 0.8000, -1.5000,
                     0.0000, 0.8000, -1.5000,
                     0.0000, 0.8000, -1.5000,
                     0.0000, 0.8000, -1.5000};

    dof_pos_rl.resize(12); // 强化学习的默认位置
    dof_pos_rl = {0.0000, 0.8000, -1.5000,
                  0.0000, 0.8000, -1.5000,
                  0.0000, 0.8000, -1.5000,
                  0.0000, 0.8000, -1.5000};
    kp_.resize(12); // 机器人的位置比例系数
    for (auto &val : kp_)
        val = 100.0;

    kd_.resize(12); // 机器人的速度比例系数
    for (auto &val : kd_)
        val = 10.0;

    // 测试ONNX模型是否加载成功，进行一次推理，如果推理存在问题就会直接报错，如果没有，就会正常输出，表明模型加载成功
    std::vector<float> dummy_input = std::vector<float>(obs_dim_, 1.0);
    std::array<int64_t, 2> input_shape{1, obs_dim_};

    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info_, dummy_input.data(), obs_dim_, input_shape.data(), input_shape.size());

    auto reaction = session_.Run(Ort::RunOptions{nullptr},
                                 input_names_.data(), &input_tensor, 1,
                                 output_names_.data(), 1);

    std::cout << "ONNX policy network test success" << std::endl;

    decimation_ = 4;
}

InterfaceOnnx::~InterfaceOnnx()
{
}

void InterfaceOnnx::enter()
{
    run_cnt_ = 0;
}

void InterfaceOnnx::push_forward() // 前向推理，最后输出的是动作值(目前来说全部存储在data里面)
{
    std::vector<float> current_obs = normalize_obs();
    std::array<int64_t, 2> input_shape{1, obs_dim_}; // 输入张量形状，[1, obs_dim_]，数据类型为 float

    // 传入的参数依次为：内存信息、输入数据指针、输入数据大小、输入形状指针、输入形状大小
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info_,
        current_obs.data(), current_obs.size(),
        input_shape.data(), input_shape.size());

    std::vector<Ort::Value> inputs;               // 创建输入张量向量
    inputs.emplace_back(std::move(input_tensor)); // 避免拷贝构造

    // 参数依次为：运行选项、输入名称指针、输入张量指针、输入张量数量、输出名称指针、输出张量数量
    auto output_tensors = session_.Run(Ort::RunOptions{nullptr},
                                       input_names_.data(),
                                       inputs.data(), 1,
                                       output_names_.data(), 1);
    // // 从输出张量中提取动作数据
    action_data = output_tensors[0].GetTensorMutableData<float>();

    // 使用vector提取动作数据
    std::vector<float> action(action_data, action_data + act_dim_);
    data.obs.last_action = action;

    // 更新data中的动作数据(并做归一化处理)
    denormalize_act();

    run_cnt_++;
}
std::vector<float> InterfaceOnnx::normalize_obs()
{

    // 进行四元数的重新映射为相应的重力的投影
    std::vector<float> projected_gravity = computeGravityProjection(data.obs.gravity_vec);
    data.obs.gravity_vec_ = projected_gravity;

    // 创建一个临时变量来存储处理后的数据，不修改原始data
    std::vector<float> scaled_angular_vel = data.obs.angular_vel;
    for (auto &vel : scaled_angular_vel)
        vel = vel * omega_scale_; // 角速度缩放

    // 创建临时变量存储处理后的命令
    std::vector<float> scaled_command = data.obs.command;
    for (auto &cmd : scaled_command)
        cmd = cmd * command_scale; // 控制指令缩放

    // 创建临时变量存储处理后的关节角度
    std::vector<float> adjusted_joint_angle = data.obs.joint_angle;
    for (int i = 0; i < 12; ++i)
        adjusted_joint_angle[i] -= dof_pos_rl[i]; // 关节角度默认值(全部为当前的默认角度的偏移量)

    /*==============================================观测量进行整合===================================================*/
    std::vector<float> integrated_obs(obs_dim_);
    int idx = 0;

    // 复制缩放后的角速度 (3维)
    for (int i = 0; i < scaled_angular_vel.size(); ++i)
        integrated_obs[idx++] = scaled_angular_vel[i];

    // 复制重力向量 (3维)
    for (int i = 0; i < data.obs.gravity_vec.size(); ++i)
        integrated_obs[idx++] = projected_gravity[i];

    // 复制缩放后的控制指令 (3维)
    for (int i = 0; i < scaled_command.size(); ++i)
        integrated_obs[idx++] = scaled_command[i];

    // 复制调整后的关节角度 (12维)
    for (int i = 0; i < adjusted_joint_angle.size(); ++i)
        integrated_obs[idx++] = adjusted_joint_angle[i];

    // 复制关节速度 (12维)
    for (int i = 0; i < data.obs.joint_vel.size(); ++i)
        integrated_obs[idx++] = data.obs.joint_vel[i];

    // 复制上一个动作 (12维)
    for (int i = 0; i < data.obs.last_action.size(); ++i)
        integrated_obs[idx++] = data.obs.last_action[i];

    return integrated_obs;
}

void InterfaceOnnx::denormalize_act()
{
    for (size_t i = 0; i < data.act.action.size(); i++)
    {
        data.act.action[i] *= action_scale_robot[i];
        data.act.action[i] += dof_pos_rl[i];
    }
}

std::vector<float> InterfaceOnnx::computeGravityProjection(const std::vector<float> &quaternion)
{
    // 四元数格式: [w, x, y, z]，表示从世界坐标系到机器人坐标系的旋转
    // 重力在世界坐标系中的向量为 [0, 0, -1]
    // 我们需要计算这个向量在机器人坐标系中的表示

    if (quaternion.size() < 4)
    {
        // 如果四元数无效，返回默认重力向量
        return {0.0f, 0.0f, -1.0f};
    }

    float w = quaternion[0];
    float x = quaternion[1];
    float y = quaternion[2];
    float z = quaternion[3];

    // 归一化四元数
    float norm = sqrt(w * w + x * x + y * y + z * z);
    if (norm > 0.0f)
    {
        w /= norm;
        x /= norm;
        y /= norm;
        z /= norm;
    }

    // 计算旋转矩阵
    float r11 = 1 - 2 * y * y - 2 * z * z;
    float r12 = 2 * x * y - 2 * z * w;
    float r13 = 2 * x * z + 2 * y * w;

    float r21 = 2 * x * y + 2 * z * w;
    float r22 = 1 - 2 * x * x - 2 * z * z;
    float r23 = 2 * y * z - 2 * x * w;

    float r31 = 2 * x * z - 2 * y * w;
    float r32 = 2 * y * z + 2 * x * w;
    float r33 = 1 - 2 * x * x - 2 * y * y;

    // 将世界坐标系下的重力向量 [0, 0, -1] 转换到机器人坐标系
    std::vector<float> projected_gravity(3);
    projected_gravity[0] = r13 * -1.0f;
    projected_gravity[1] = r23 * -1.0f;
    projected_gravity[2] = r33 * -1.0f;

    return projected_gravity;
}
void InterfaceOnnx::set_obs(robot_msgs::RobotState robot_state, std::vector<float> command) // 将传感器传入的参数全部输入到策略网络中，将data进行更新
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
std::vector<float> InterfaceOnnx::get_act() // 获取策略网络的输出（关节的绝对值参数）
{
    return data.act.action;
}