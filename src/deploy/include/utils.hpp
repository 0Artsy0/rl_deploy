#pragma once

#include <vector>
#include <iostream>
#include <robot_param.hpp>

bool calculate_target_joint_pos(std::vector<float> &pose,std::vector<float> &init_pose, std::vector<float> &target_pos,int &count,int during)//差分函数
{
    float percent=static_cast<float>(count)/static_cast<float>(during);

    if(init_pose.size()!=target_pos.size())//安全检查
    {
        std::cerr << "Error: init_pose and target_pos size mismatch!" << std::endl;
        return false;
    }

    for(size_t i=0;i<init_pose.size();i++)//差值处理
    {
        pose[i]=init_pose[i]*(1.0f - percent)+target_pos[i]*percent;
    }

    count>during?count:count++;
    
    if(count>during)
        return true;
    else
        return false;
}

bool joint_safety_check(std::vector<float> &joint_pos,std::vector<float> &joint_vel)//关节速度和位置安全检查
{

    robot_param _robot_param;
    int single_joint_num = joint_pos.size()/4;

    if (joint_pos.size() != joint_vel.size()) {
        std::cerr << "Error: joint_pos and joint_vel size mismatch!" << std::endl;
        return false;
    }
    
    for (size_t j = 0; j < 4; j++){
        for (size_t i = 0; i < single_joint_num; i++) {
            if (joint_pos[j*single_joint_num+i] < _robot_param.joint_limit[i][0] || joint_pos[j*single_joint_num+i] > _robot_param.joint_limit[i][1]) {
                std::cerr << "Warning: Joint " << j*single_joint_num+i << " position out of range: " << joint_pos[j*single_joint_num+i] << std::endl;
                return false;
            }
        }
    }

    for(size_t i = 0; i < joint_vel.size(); i++){
         if(std::abs(joint_vel[i])>_robot_param.Max_speed){
            std::cerr << "Warning: Joint " << i << " velocity too high: " << joint_vel[i] << " rad/s" << std::endl;
            return false;
        }
    }
    
    return true;
}

bool body_safety_check(std::vector<float> &gravity_projection)//机身姿态安全检查
{
        if(gravity_projection[2]>-0.7){
            std::cout << "Warning: Body is not stable, gravity projection: " << std::endl;
            return false;
        }
        else
            return true;
}