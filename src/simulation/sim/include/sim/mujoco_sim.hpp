#ifndef MUJOCO_SIM_HPP_
#define MUJOCO_SIM_HPP_

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <vector>
#include <atomic>

#include <mujoco/mujoco.h>
#include <mujoco/mujoco_utils.hpp>
#include <ros/ros.h>
#include <robot_msgs/RobotState.h>
#include <robot_msgs/RobotCommand.h>
#include <robot_msgs/MotorCommand.h>
#include <robot_msgs/MotorState.h>

class Mujoco_Sim
{
private:
  bool running = false;
  // ros
  ros::NodeHandle nh;
  ros::Publisher robot_state_pub;    // 机器人状态发布者
  ros::Subscriber robot_command_sub; // 机器人控制指令订阅者

  std::vector<std::string> robot_State_topic = {
      "/go2_mujoco/robot_state"};

  std::vector<std::string> robot_Command_topic = {
      "/go2_mujoco/robot_command"};

  // mujoco
  mjvCamera cam;
  mjvOption opt;
  mjvPerturb pert;

  mjModel *mj_model = nullptr;
  mjData *mj_data = nullptr;
  std::unique_ptr<mj::Simulate> sim; // 保存sim对象以便在析构函数中访问
  std::mutex data_mutex;             // 用于保护数据访问的互斥锁
  std::vector<double> mujoco_torque; //  mujoco 关节力矩

  const char *filename = ROBOT_DESCRIPTION_PATH "/go2/go2_mjcf/scene.xml";

  // thread
  std::thread physicsthreadhandle; // 物理仿真线程
  std::thread ros_threadhandle;    // 渲染线程
  std::thread test;

  // 关节
  robot_msgs::RobotCommand robot_command; // 12个关节的控制指令
  robot_msgs::RobotState robot_state;     // 12个关节的状态
  robot_msgs::MotorCommand joint_command;
  robot_msgs::MotorState joint_state;

public:
  Mujoco_Sim();
  ~Mujoco_Sim();

  void ros_thread();
  void ros_thread_init();

  void joint_State();   // 状态处理
  void joint_Command(); // 传入关节控制指令

  void robot_pub();                                              // 发布机器人状态
  void robot_sub(const robot_msgs::RobotCommand::ConstPtr &msg); // 接受机器人控制指令

  void print_joint_State();
};

#endif