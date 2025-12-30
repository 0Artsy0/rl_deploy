cmake_minimum_required(VERSION 3.10)
project(Joy_SDK)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

# 添加可执行文件
add_executable(joystick_reader src/Joy_interface.cpp src/joy_stick_test.cpp)

# 包含头文件目录
target_include_directories(joystick_reader PUBLIC include)

# 链接必要的库
target_link_libraries(joystick_reader Threads::Threads)