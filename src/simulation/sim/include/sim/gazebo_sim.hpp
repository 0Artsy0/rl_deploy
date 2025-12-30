#ifndef GAZEBO_SIM_HPP_
#define GAZEBO_SIM_HPP_

#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/Imu.h>
#include <vector>
#include <mutex>

#include <robot_msgs/RobotState.h>
#include <robot_msgs/RobotCommand.h>
#include <robot_msgs/MotorState.h>
#include <robot_msgs/MotorCommand.h>

class Gazebo_Sim
{
private:
    ros::NodeHandle nh;

    ros::Subscriber sub[12];        // 订阅关节状态
    ros::Publisher pub[12];         // 发布关节控制命令
    ros::Subscriber sub_robot_cmd;  // 机器人控制指令订阅
    ros::Publisher pub_robot_state; // 机器人状态发布

    ros::Subscriber imu_state_sub; // 接受传感器数据

    robot_msgs::RobotState robot_state;     // 机器人状态
    robot_msgs::RobotCommand robot_cmd;     // 机器人指令
    robot_msgs::MotorState motor_state[12]; // 电机状态
    robot_msgs::MotorCommand motor_cmd[12]; // 电机指令
    sensor_msgs::Imu robot_imu_state;       // imu状态

    std::vector<std::string> pub_topic = {
        "/go2_gazebo/FR_hip_controller/command",
        "/go2_gazebo/FR_thigh_controller/command",
        "/go2_gazebo/FR_calf_controller/command",
        "/go2_gazebo/FL_hip_controller/command",
        "/go2_gazebo/FL_thigh_controller/command",
        "/go2_gazebo/FL_calf_controller/command",
        "/go2_gazebo/RR_hip_controller/command",
        "/go2_gazebo/RR_thigh_controller/command",
        "/go2_gazebo/RR_calf_controller/command",
        "/go2_gazebo/RL_hip_controller/command",
        "/go2_gazebo/RL_thigh_controller/command",
        "/go2_gazebo/RL_calf_controller/command"};

    std::vector<std::string> sub_topic = {
        "/go2_gazebo/FR_hip_controller/state",
        "/go2_gazebo/FR_thigh_controller/state",
        "/go2_gazebo/FR_calf_controller/state",
        "/go2_gazebo/FL_hip_controller/state",
        "/go2_gazebo/FL_thigh_controller/state",
        "/go2_gazebo/FL_calf_controller/state",
        "/go2_gazebo/RR_hip_controller/state",
        "/go2_gazebo/RR_thigh_controller/state",
        "/go2_gazebo/RR_calf_controller/state",
        "/go2_gazebo/RL_hip_controller/state",
        "/go2_gazebo/RL_thigh_controller/state",
        "/go2_gazebo/RL_calf_controller/state"};

    std::vector<std::string> imu_topic = {
        "/trunk_imu"};

    std::vector<std::string> robot_State_topic = {
        "/go2_gazebo/robot_state"};

    std::vector<std::string> robot_Command_topic = {
        "/go2_gazebo/robot_command"};

    std::mutex lock;

public:
    Gazebo_Sim();
    ~Gazebo_Sim();

    // 控制器回调函数
    void FL_Hip_joint_State_(const robot_msgs::MotorState::ConstPtr &msg);
    void FL_Thigh_joint_State_(const robot_msgs::MotorState::ConstPtr &msg);
    void FL_Calf_joint_State_(const robot_msgs::MotorState::ConstPtr &msg);
    void FR_Hip_joint_State_(const robot_msgs::MotorState::ConstPtr &msg);
    void FR_Thigh_joint_State_(const robot_msgs::MotorState::ConstPtr &msg);
    void FR_Calf_joint_State_(const robot_msgs::MotorState::ConstPtr &msg);
    void RL_Hip_joint_State_(const robot_msgs::MotorState::ConstPtr &msg);
    void RL_Thigh_joint_State_(const robot_msgs::MotorState::ConstPtr &msg);
    void RL_Calf_joint_State_(const robot_msgs::MotorState::ConstPtr &msg);
    void RR_Hip_joint_State_(const robot_msgs::MotorState::ConstPtr &msg);
    void RR_Thigh_joint_State_(const robot_msgs::MotorState::ConstPtr &msg);
    void RR_Calf_joint_State_(const robot_msgs::MotorState::ConstPtr &msg);

    // 传感器回调函数
    void imu_state_Callback(const sensor_msgs::Imu::ConstPtr &msg);

    // 机器人指令回调函数
    void robot_Command_Callback(const robot_msgs::RobotCommand::ConstPtr &msg);

    // 机器人状态回调函数
    void robot_pub(void); // 机器人状态发布函数
};

#endif