#pragma once

#include "../rl_data_operatation.hpp"
#include "iostream"
#include "vector"
#include <onnxruntime_cxx_api.h>
#include <cmath>

class InterfaceOnnx : public RLDataOperation
{
private:
    std::string _model_path; // 模型的文件路径
    std::string _device;     // 模型推理的硬件平台，包含CPU或者GPU

    // ONNX Runtime相关成员
    Ort::Env _env;                       // onnxruntime环境
    Ort::SessionOptions _session_options; // 会话选项
    Ort::MemoryInfo _memory_info;        // onnxruntime内存信息
    Ort::Session _session;               // onnxruntime会话

    float* action_data;    

    std::vector<const char*> _input_names;  // 输入节点名称
    std::vector<const char*> _output_names; // 输出节点名称

public:
    InterfaceOnnx(const std::string &model_path); // 第一个参数是模型的文件路径，第二个参数是模型推理的硬件平台，包含CPU或者GPU
    ~InterfaceOnnx() noexcept;

    void push_forward() override;
};


InterfaceOnnx::InterfaceOnnx(const std::string &model_path)
    : _model_path(model_path),
      _device("CPUExecutionProvider"),
      _env(ORT_LOGGING_LEVEL_WARNING, "policy_forward"),
      _session_options(),
      _memory_info(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)),
      _session(nullptr)
{
    // 设置会话选项
    _session_options.SetIntraOpNumThreads(1);
    _session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    _session = Ort::Session(_env, _model_path.c_str(), _session_options);

    _input_names.push_back(data.obs_group.c_str());  // 输入节点的名称
    _output_names.push_back(data.act_group.c_str()); // 输出节点的名称

    // 测试ONNX模型是否加载成功，进行一次推理，如果推理存在问题就会直接报错，如果没有，就会正常输出，表明模型加载成功
    std::vector<float> dummy_input = std::vector<float>(_robot_param._obs_dim, 1.0);
    std::array<int64_t, 2> input_shape{1, _robot_param._obs_dim};

    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        _memory_info, dummy_input.data(), _robot_param._obs_dim, input_shape.data(), input_shape.size());

    auto reaction = _session.Run(Ort::RunOptions{nullptr},
                                 _input_names.data(), &input_tensor, 1,
                                 _output_names.data(), 1);

    std::cout << "ONNX policy network test success" << std::endl;

}

InterfaceOnnx::~InterfaceOnnx()
{
}

void InterfaceOnnx::push_forward() // 前向推理，最后输出的是动作值(目前来说全部存储在data里面)
{
    std::vector<float> current_obs = set_obs(); // 获取归一化后的观测量
    std::array<int64_t, 2> input_shape{1, _robot_param._obs_dim}; // 输入张量形状，[1, obs_dim_]，数据类型为 float

    // 传入的参数依次为：内存信息、输入数据指针、输入数据大小、输入形状指针、输入形状大小
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        _memory_info,
        current_obs.data(), current_obs.size(),
        input_shape.data(), input_shape.size());

    std::vector<Ort::Value> inputs;               // 创建输入张量向量
    inputs.emplace_back(std::move(input_tensor)); // 避免拷贝构造

    // 参数依次为：运行选项、输入名称指针、输入张量指针、输入张量数量、输出名称指针、输出张量数量
    auto output_tensors = _session.Run(Ort::RunOptions{nullptr},
                                       _input_names.data(),
                                       inputs.data(), 1,
                                       _output_names.data(), 1);
    // 从输出张量中提取动作数据
    action_data = output_tensors[0].GetTensorMutableData<float>();

    // 使用vector提取动作数据
    std::vector<float> action(action_data, action_data + _robot_param._act_dim);
    data.obs.last_action = action;

    // 更新data中的动作数据(并做归一化处理)
    set_act(action);

}
