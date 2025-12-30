#include <ros/ros.h>
#include "FSM/FSM_Manager.hpp"
#include <csignal> // 包含信号处理函数头文件

// 声明全局指针，用于信号处理函数访问
FSM_Manager* g_fsm_manager = nullptr;

std::string rl_model;//rl推理模型
std::string sim_engine;//sim2sim仿真模型

// 自定义终端信号处理函数
void customShutdownHandler(int sig) {
    ROS_INFO("SIGINT signal received. Shutting down...");
    ROS_INFO("正在执行自定义清理操作...");

    // 检查全局指针是否有效
    if (g_fsm_manager != nullptr) {
        g_fsm_manager->running = false; // 停止状态机运行
        ROS_INFO("状态机已停止运行");
    } else {
        ROS_WARN("状态机指针为空，无法停止状态机");
    }

    ROS_INFO("清理完成,正在关闭ROS系统...");
    ros::shutdown();
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
/* =====================选择仿真引擎==============================*/
    if (argc!=3)
    {
        ROS_ERROR("请输入仿真引擎参数以及模型类型，例如：./rl_deploy gazebo onnx");
        return 1;
    }
    else 
    {
        sim_engine = argv[1];
        rl_model = argv[2];
    }
    
    // 注册终端信号处理函数
    signal(SIGINT, customShutdownHandler);
    ros::init(argc, argv, "go2_deploy");
    
    // 创建状态机管理器
    FSM_Manager Robot_Deploy(sim_engine,rl_model);
    
    // 将对象地址赋给全局指针
    g_fsm_manager = &Robot_Deploy;

    // 运行状态机
    Robot_Deploy.run();

    // 程序退出前清理全局指针
    g_fsm_manager = nullptr;
    
    return 0;
}