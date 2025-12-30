cmake_minimum_required(VERSION 3.5)
project(go2_urdf VERSION 3.0.0)

# ROS1 (Noetic) configuration
find_package(catkin REQUIRED COMPONENTS
    urdf
    xacro
    robot_state_publisher
    gazebo_ros
)

catkin_package(
    CATKIN_DEPENDS
    urdf
    xacro
    robot_state_publisher
    gazebo_ros
)