cmake_minimum_required(VERSION 3.0.2)
project(sim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(POLICY CMP0072)
  cmake_policy(SET CMP0072 NEW)
  set(OpenGL_GL_PREFERENCE "GLVND")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  robot_msgs
  sensor_msgs
  geometry_msgs
)

## 查找系统依赖
find_package(OpenGL REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GLFW REQUIRED glfw3)

set(MUJOCO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../library/mujoco")
set(MUJOCO_INCLUDE_DIR "${MUJOCO_DIR}/include")
set(MUJOCO_LIBRARY "${MUJOCO_DIR}/lib/libmujoco.so")

## 检查MuJoCo库是否存在
if(NOT EXISTS ${MUJOCO_LIBRARY})
  message(FATAL_ERROR "MuJoCo library not found at ${MUJOCO_LIBRARY}")
endif()

###################################
## catkin特定配置 ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs robot_msgs sensor_msgs geometry_msgs
  DEPENDS OpenGL GLFW
)

###########
## 构建 ##
###########

## 指定头文件的额外位置
include_directories(
  include
  ${MUJOCO_INCLUDE_DIR}
  ${MUJOCO_DIR}/simulate
  ${catkin_INCLUDE_DIRS}
  ${OpenGL_INCLUDE_DIRS}
  ${GLFW_INCLUDE_DIRS}
)

## 声明MuJoCo仿真可执行文件
add_executable(sim2sim_mujoco
  src/mujoco_sim.cpp
  include/mujoco/glfw_adapter.cc
  include/mujoco/platform_ui_adapter.cc
  include/mujoco/simulate.cc
  include/mujoco/glfw_dispatch.cc
)

## 声明Gazebo仿真可执行文件
add_executable(sim2sim_gazebo
  src/gazebo_sim.cpp
)

## 重命名可执行文件（可选，确保输出名称正确）
set_target_properties(sim2sim_mujoco PROPERTIES OUTPUT_NAME sim2sim_mujoco)
set_target_properties(sim2sim_gazebo PROPERTIES OUTPUT_NAME sim2sim_gazebo)

## 添加cmake目标依赖
add_dependencies(sim2sim_mujoco ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(sim2sim_gazebo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 指定要链接的库
target_link_libraries(sim2sim_mujoco
  ${MUJOCO_LIBRARY}
  ${GLFW_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenGL_LIBRARIES}
  pthread
)

target_link_libraries(sim2sim_gazebo
  ${catkin_LIBRARIES}
)

# 设置机器人描述文件路径
set(ROBOT_DESCRIPTION_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../robot_description")
add_definitions(-DROBOT_DESCRIPTION_PATH="${ROBOT_DESCRIPTION_PATH}")