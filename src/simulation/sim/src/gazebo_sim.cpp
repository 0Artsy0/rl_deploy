#include "sim/gazebo_sim.hpp"

Gazebo_Sim::Gazebo_Sim()
{
    robot_cmd.motor_command.resize(12);
    robot_state.motor_state.resize(12);

    for (int i = 0; i < 12; i++) // 发布关节控制命令
    {
        pub[i] = nh.advertise<robot_msgs::MotorCommand>(pub_topic[i], 10);
    }
    // 订阅关节状态
    sub[0] = nh.subscribe<robot_msgs::MotorState>(sub_topic[3], 10, &Gazebo_Sim::FR_Hip_joint_State_, this);
    sub[1] = nh.subscribe<robot_msgs::MotorState>(sub_topic[4], 10, &Gazebo_Sim::FR_Thigh_joint_State_, this);
    sub[2] = nh.subscribe<robot_msgs::MotorState>(sub_topic[5], 10, &Gazebo_Sim::FR_Calf_joint_State_, this);
    sub[3] = nh.subscribe<robot_msgs::MotorState>(sub_topic[0], 10, &Gazebo_Sim::FL_Hip_joint_State_, this);
    sub[4] = nh.subscribe<robot_msgs::MotorState>(sub_topic[1], 10, &Gazebo_Sim::FL_Thigh_joint_State_, this);
    sub[5] = nh.subscribe<robot_msgs::MotorState>(sub_topic[2], 10, &Gazebo_Sim::FL_Calf_joint_State_, this);
    sub[6] = nh.subscribe<robot_msgs::MotorState>(sub_topic[9], 10, &Gazebo_Sim::RR_Hip_joint_State_, this);
    sub[7] = nh.subscribe<robot_msgs::MotorState>(sub_topic[10], 10, &Gazebo_Sim::RR_Thigh_joint_State_, this);
    sub[8] = nh.subscribe<robot_msgs::MotorState>(sub_topic[11], 10, &Gazebo_Sim::RR_Calf_joint_State_, this);
    sub[9] = nh.subscribe<robot_msgs::MotorState>(sub_topic[6], 10, &Gazebo_Sim::RL_Hip_joint_State_, this);
    sub[10] = nh.subscribe<robot_msgs::MotorState>(sub_topic[7], 10, &Gazebo_Sim::RL_Thigh_joint_State_, this);
    sub[11] = nh.subscribe<robot_msgs::MotorState>(sub_topic[8], 10, &Gazebo_Sim::RL_Calf_joint_State_, this);

    // 订阅imu状态
    imu_state_sub = nh.subscribe<sensor_msgs::Imu>(imu_topic[0], 10, &Gazebo_Sim::imu_state_Callback, this);
    // 订阅机器人控制指令
    sub_robot_cmd = nh.subscribe<robot_msgs::RobotCommand>(robot_Command_topic[0], 10, &Gazebo_Sim::robot_Command_Callback, this);
    // 发布机器人状态
    pub_robot_state = nh.advertise<robot_msgs::RobotState>(robot_State_topic[0], 10);
}

Gazebo_Sim::~Gazebo_Sim()
{
}

// imu状态回调函数
void Gazebo_Sim::imu_state_Callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    lock.lock();
    robot_imu_state = *msg;
    lock.unlock();
}

// 机器人控制指令回调函数
void Gazebo_Sim::robot_Command_Callback(const robot_msgs::RobotCommand::ConstPtr &msg)
{
    lock.lock();
    robot_cmd = *msg;
    lock.unlock();
}
/*========================================右前腿=================================================*/
void Gazebo_Sim::FR_Hip_joint_State_(const robot_msgs::MotorState::ConstPtr &msg)
{
    lock.lock();
    robot_state.motor_state[0] = *msg;
    lock.unlock();
}

void Gazebo_Sim::FR_Thigh_joint_State_(const robot_msgs::MotorState::ConstPtr &msg)
{
    lock.lock();
    robot_state.motor_state[1] = *msg;
    lock.unlock();
}

void Gazebo_Sim::FR_Calf_joint_State_(const robot_msgs::MotorState::ConstPtr &msg)
{
    lock.lock();
    robot_state.motor_state[2] = *msg;
    lock.unlock();
}
/*========================================左前腿=================================================*/
void Gazebo_Sim::FL_Hip_joint_State_(const robot_msgs::MotorState::ConstPtr &msg)
{
    lock.lock();
    robot_state.motor_state[3] = *msg;
    lock.unlock();
}

void Gazebo_Sim::FL_Thigh_joint_State_(const robot_msgs::MotorState::ConstPtr &msg)
{
    lock.lock();
    robot_state.motor_state[4] = *msg;
    lock.unlock();
}

void Gazebo_Sim::FL_Calf_joint_State_(const robot_msgs::MotorState::ConstPtr &msg)
{
    lock.lock();
    robot_state.motor_state[5] = *msg;
    lock.unlock();
}
/*========================================右后腿=================================================*/
void Gazebo_Sim::RR_Hip_joint_State_(const robot_msgs::MotorState::ConstPtr &msg)
{
    lock.lock();
    robot_state.motor_state[6] = *msg;
    lock.unlock();
}

void Gazebo_Sim::RR_Thigh_joint_State_(const robot_msgs::MotorState::ConstPtr &msg)
{
    lock.lock();
    robot_state.motor_state[7] = *msg;
    lock.unlock();
}

void Gazebo_Sim::RR_Calf_joint_State_(const robot_msgs::MotorState::ConstPtr &msg)
{
    lock.lock();
    robot_state.motor_state[8] = *msg;
    lock.unlock();
}
/*========================================左后腿=================================================*/
void Gazebo_Sim::RL_Hip_joint_State_(const robot_msgs::MotorState::ConstPtr &msg)
{
    lock.lock();
    robot_state.motor_state[9] = *msg;
    lock.unlock();
}

void Gazebo_Sim::RL_Thigh_joint_State_(const robot_msgs::MotorState::ConstPtr &msg)
{
    lock.lock();
    robot_state.motor_state[10] = *msg;
    lock.unlock();
}

void Gazebo_Sim::RL_Calf_joint_State_(const robot_msgs::MotorState::ConstPtr &msg)
{
    lock.lock();
    robot_state.motor_state[11] = *msg;
    lock.unlock();
}

// 机器人状态发布函数
void Gazebo_Sim::robot_pub(void)
{
    // 添加imu状态
    robot_state.imu.quaternion[0] = robot_imu_state.orientation.w; // 四元数
    robot_state.imu.quaternion[1] = robot_imu_state.orientation.x;
    robot_state.imu.quaternion[2] = robot_imu_state.orientation.y;
    robot_state.imu.quaternion[3] = robot_imu_state.orientation.z;
    robot_state.imu.accelerometer[0] = robot_imu_state.linear_acceleration.x; // 加速度
    robot_state.imu.accelerometer[1] = robot_imu_state.linear_acceleration.y;
    robot_state.imu.accelerometer[2] = robot_imu_state.linear_acceleration.z;
    robot_state.imu.gyroscope[0] = robot_imu_state.angular_velocity.x; // 陀螺仪
    robot_state.imu.gyroscope[1] = robot_imu_state.angular_velocity.y;
    robot_state.imu.gyroscope[2] = robot_imu_state.angular_velocity.z;

    pub_robot_state.publish(robot_state); // 发布机器人状态

    for (int i = 0; i < 12; i++)
    {
        pub[i].publish(robot_cmd.motor_command[i]); // 关节控制指令
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sim2sim_gazebo");

    Gazebo_Sim gazebo_sim;

    ros::Rate rate(500);
    while (ros::ok())
    {
        gazebo_sim.robot_pub();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}