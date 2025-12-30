cmake_minimum_required(VERSION 3.0.2)
project(deploy)

## 设置C++标准为C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## 查找catkin包和依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  robot_msgs
)

## 声明catkin包
catkin_package(
  LIBRARIES ${PROJECT_NAME}_torchscript ${PROJECT_NAME}_onnx
  CATKIN_DEPENDS roscpp rospy std_msgs robot_msgs
  DEPENDS libtorch onnxruntime
)

## 查找系统依赖
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)  # 添加线程库依赖

## 查找libtorch
set(Torch_DIR /home/lym/zhang/deploy/src/library/libtorch/share/cmake/Torch)
find_package(Torch REQUIRED)

## 查找onnxruntime
set(ONNXRUNTIME_ROOT /home/lym/zhang/deploy/src/library/onnxruntime)
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_c_api.h
  PATHS ${ONNXRUNTIME_ROOT}/include
  NO_DEFAULT_PATH
)
find_library(ONNXRUNTIME_LIBRARY onnxruntime
  PATHS ${ONNXRUNTIME_ROOT}/lib
  NO_DEFAULT_PATH
)

## 包含头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ONNXRUNTIME_INCLUDE_DIR}
  ${TORCH_INCLUDE_DIRS}
  library/Joy_SDK/include  # 添加Joy_SDK头文件路径
)

## 添加库
add_library(${PROJECT_NAME}_torchscript
  include/PPO/Torchscript/interface_torchscript.cpp
)

add_library(${PROJECT_NAME}_onnx
  include/PPO/onnx/interface_onnx.cpp
)

## 添加Joy_SDK库
add_library(${PROJECT_NAME}_joy
  library/Joy_SDK/src/Joy_interface.cpp
)

## 添加可执行文件 - 将名称改为rl_deploy
add_executable(rl_deploy src/deploy.cpp)

## 链接库
target_link_libraries(${PROJECT_NAME}_torchscript
  ${catkin_LIBRARIES}
  ${TORCH_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_onnx
  ${catkin_LIBRARIES}
  ${ONNXRUNTIME_LIBRARY}
)

target_link_libraries(${PROJECT_NAME}_joy
  Threads::Threads  # JoyInterface需要线程库
)

target_link_libraries(rl_deploy
  ${PROJECT_NAME}_torchscript
  ${PROJECT_NAME}_onnx
  ${PROJECT_NAME}_joy  # 添加Joy_SDK库
  ${catkin_LIBRARIES}
  ${TORCH_LIBRARIES}
  ${ONNXRUNTIME_LIBRARY}
  Threads::Threads  # 添加线程库
)

## 添加依赖
add_dependencies(${PROJECT_NAME}_torchscript ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_onnx ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_joy ${catkin_EXPORTED_TARGETS})
add_dependencies(rl_deploy ${catkin_EXPORTED_TARGETS})

## 设置目标属性
set_target_properties(${PROJECT_NAME}_torchscript PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)

set_target_properties(${PROJECT_NAME}_onnx PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)

set_target_properties(${PROJECT_NAME}_joy PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)

set_target_properties(rl_deploy PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)

## 安装目标
install(TARGETS ${PROJECT_NAME}_torchscript ${PROJECT_NAME}_onnx ${PROJECT_NAME}_joy
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS rl_deploy
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 安装头文件
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)