#include "sim/mujoco_sim.hpp"

Mujoco_Sim::Mujoco_Sim()
{
    running = true;

    robot_state.motor_state.resize(12);
    robot_command.motor_command.resize(12);
    mujoco_torque.resize(12);

    // mujoco
    if (mjVERSION_HEADER != mj_version())
    {
        mju_error("Headers and library have different versions");
    }

    scanPluginLibraries();

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultPerturb(&pert);

    sim = std::make_unique<mj::Simulate>(
        std::make_unique<mj::GlfwAdapter>(),
        &cam, &opt, &pert, /* is_passive = */ false);

    physicsthreadhandle = std::thread(&PhysicsThread, sim.get(), filename);

    test = std::thread(&Mujoco_Sim::print_joint_State, this);

    // 获取mujoco中模型和数据
    while (1)
    {
        if (d)
        {
            std::cout << "[INFO] [MuJoCo] Data prepared" << std::endl;

            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    this->mj_model = m;
    this->mj_data = d;

    ros_thread_init();

    sim->RenderLoop();
}

Mujoco_Sim::~Mujoco_Sim()
{
    running = false;

    if (sim)
    {
        sim->exitrequest.store(true);
    }

    // 关闭物理仿真线程
    if (physicsthreadhandle.joinable())
    {
        physicsthreadhandle.join();
    }
    // // 关闭测试线程
    if (test.joinable())
    {
        test.join();
    }
    // 关闭ros线程
    if (ros_threadhandle.joinable())
    {
        ros_threadhandle.join();
    }
}

void Mujoco_Sim::ros_thread_init()
{
    // 初始化机器人状态发布者
    robot_state_pub = nh.advertise<robot_msgs::RobotState>(robot_State_topic[0], 10);
    // 初始化机器人控制指令订阅者
    robot_command_sub = nh.subscribe<robot_msgs::RobotCommand>(robot_Command_topic[0], 10, &Mujoco_Sim::robot_sub, this);

    ros_threadhandle = std::thread(&Mujoco_Sim::ros_thread, this);
}
void Mujoco_Sim::ros_thread()
{
    ros::Rate loop_rate(100);
    while (running)
    {
        joint_Command();
        joint_State();
        robot_state_pub.publish(robot_state);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void Mujoco_Sim::robot_sub(const robot_msgs::RobotCommand::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(data_mutex);
    robot_command = *msg;
}

void Mujoco_Sim::joint_Command() // 传入关节控制指令
{
    std::lock_guard<std::mutex> lock(data_mutex);

    if (mj_data )
    {
        for (int i = 0; i < 12; i++)
        {
            mujoco_torque[i] = (robot_command.motor_command[i].q - mj_data->sensordata[i]) * robot_command.motor_command[i].kp + (robot_command.motor_command[i].dq - mj_data->sensordata[i + 12]) * robot_command.motor_command[i].kd + robot_command.motor_command[i].tau; // 计算关节力矩
            mj_data->ctrl[i] = mujoco_torque[i];                                                                                                                                                                                                                    // 设置关节力矩
        }
    }
}

void Mujoco_Sim::joint_State() // 获取关节状态
{
    std::lock_guard<std::mutex> lock(data_mutex);

    if (mj_data && mj_model)
    {
        // 从mj_data中获取关节位置和速度
        for (int i = 0; i < 12; i++)
        {
            robot_state.motor_state[i].q = mj_data->sensordata[i];            // 关节位置
            robot_state.motor_state[i].dq = mj_data->sensordata[i + 12];      // 关节速度
            robot_state.motor_state[i].tau_est = mj_data->sensordata[i + 24]; // 关节力矩
        }
        // imu的四元数、陀螺仪、加速度计
        for (int i = 0; i < 4; i++)
        {
            robot_state.imu.quaternion[i] = mj_data->sensordata[36 + i]; // 四元数
        }
        for (int i = 0; i < 3; i++)
        {
            robot_state.imu.gyroscope[i] = mj_data->sensordata[40 + i];     // 陀螺仪
            robot_state.imu.accelerometer[i] = mj_data->sensordata[43 + i]; // 加速度计
        }
    }
}

void Mujoco_Sim::print_joint_State()
{
    while (running)
    {
        // if (mj_data && mj_data->qpos && mj_model && mj_model->njnt > 4)
        // {
        //     // std::cout << "joint_state: " << mj_data->qpos[8] << std::endl;
        //     // std::cout << "joint_state_3: " << mj_data->sensordata[8] << std::endl;
        // }
        // else
        // {
        //     std::cout << "Data not ready or insufficient joints" << std::endl;
        // }
        // std::cout<<"关节力矩1为"<<mujoco_torque[0]<<std::endl;
        // std::cout<<"关节速度1为"<<robot_state.motor_state[0].dq<<std::endl;
        // std::cout<<"关节力矩2为"<<mujoco_torque[1]<<std::endl;
        // std::cout<<"关节速度2为"<<robot_state.motor_state[1].dq<<std::endl;
        // std::cout<<"关节力矩3为"<<mujoco_torque[2]<<std::endl;
        // std::cout<<"关节速度3为"<<robot_state.motor_state[2].dq<<std::endl;

        usleep(100000); // 100ms
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sim2sim_mujoco");
    Mujoco_Sim mujoco_sim;

    return 0;
}