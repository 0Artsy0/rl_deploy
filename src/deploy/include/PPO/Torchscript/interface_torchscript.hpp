#pragma once

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
    std::string _model_path; // 模型的文件路径
    std::string _device;     // 模型推理的硬件平台，包含CPU或者GPU

    torch::jit::script::Module model; // TorchScript模块

public:
    InterfaceTorchscript(const std::string &model_path); // 第一个参数是模型的文件路径，第二个参数是模型推理的硬件平台，包含CPU或者GPU
    ~InterfaceTorchscript() noexcept;
    
    void push_forward() override;
};


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

void InterfaceTorchscript::push_forward()// 策略网络前向推理
{
    try
    {
        // Convert observation vector to Torch tensor
        std::vector<float> current_obs = set_obs();

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

        set_act(act);//动作的缩放映射
        // Store action in data

        // data.obs.last_action = act;
    }
    catch (const std::exception &e) // 捕获Torch推理过程中可能抛出的异常
    {
        std::cout << "Torch inference error: " << e.what() << std::endl;
    }
}

