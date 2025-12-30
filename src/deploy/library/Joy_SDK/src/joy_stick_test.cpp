#include "Joy_interface.hpp"
#include "iostream"
#include "unistd.h"
#include "chrono"
#include "thread"

int main() {
    // 1. 初始化手柄接口（替换为你的实际设备节点，如 /dev/input/js1）
    JoyInterface joy(DEFAULT_JOY_PORT);

    // 2. 检查连接状态
    if (!joy.IsConnected()) {
        std::cerr << "程序启动失败：手柄未连接或设备节点错误" << std::endl;
        return 1;
    }

    std::cout << "手柄测试程序已启动，开始读取数据..." << std::endl;
    std::cout << "---------------------------------------------------------------------------------" << std::endl;

    // 3. 定时打印配置（50ms一次，平衡实时性和视觉效果）
    auto last_print_time = std::chrono::steady_clock::now();
    const std::chrono::milliseconds print_interval = std::chrono::milliseconds(PRINT_INTERVAL_MS);

    // 4. 主循环：批量处理事件 + 定时清屏打印
    while (true) {
        // 定时清屏打印（仅当状态变化或到达打印间隔时打印）
        auto now = std::chrono::steady_clock::now();
        if (joy.IsStateChanged() || 
            std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print_time) >= print_interval) {
            
            system("clear");
            joy.PrintState();
            last_print_time = now;
        }

        // 1ms休眠：降低CPU占用，同时保证事件处理实时性
        //usleep(1000);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return 0;
}