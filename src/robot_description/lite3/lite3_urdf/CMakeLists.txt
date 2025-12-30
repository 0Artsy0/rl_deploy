cmake_minimum_required(VERSION 3.5)
project(lite3_urdf VERSION 3.0.0)

# ROS 1配置
find_package(catkin REQUIRED COMPONENTS
    urdf
    xacro
    robot_state_publisher
    gazebo_ros
)

catkin_package(
    CATKIN_DEPENDS urdf xacro robot_state_publisher gazebo_ros
)

# 安装目录
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY meshes/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)
install(DIRECTORY urdf/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)
install(DIRECTORY xacro/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/xacro)