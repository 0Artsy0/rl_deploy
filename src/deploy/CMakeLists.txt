cmake_minimum_required(VERSION 3.0.2)
project(deploy)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  robot_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs robot_msgs
  DEPENDS libtorch onnxruntime
)

find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)

set(Torch_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../library/libtorch/share/cmake/Torch)
find_package(Torch REQUIRED)

set(ONNXRUNTIME_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../library/onnxruntime)
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_c_api.h
  PATHS ${ONNXRUNTIME_ROOT}/include
  NO_DEFAULT_PATH
)
find_library(ONNXRUNTIME_LIBRARY onnxruntime
  PATHS ${ONNXRUNTIME_ROOT}/lib
  NO_DEFAULT_PATH
)

include_directories(
  include
  ${TORCH_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${ONNXRUNTIME_INCLUDE_DIR}
)

add_executable(rl_deploy src/deploy.cpp)

target_link_libraries(rl_deploy
  ${TORCH_LIBRARIES}
  ${catkin_LIBRARIES}
  ${ONNXRUNTIME_LIBRARY}
  Threads::Threads
)

add_dependencies(rl_deploy ${catkin_EXPORTED_TARGETS})

set_target_properties(rl_deploy PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)