#pragma once


#include <iostream>
#include <ros/ros.h>
#include <robot_msgs/MotorCommand.h>
#include <robot_msgs/MotorState.h>
#include <sensor_msgs/Imu.h>
#include <boost/bind.hpp>
#include <thread>
#include <vector>

#include "hardware_interface.hpp"

#define interfaze_rate 500

class usb2can_interface : public hardware_interface
{ 

};