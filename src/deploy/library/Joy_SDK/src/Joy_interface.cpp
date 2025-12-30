#include "Joy_interface.hpp"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cerrno>
#include <cmath>
#include <iomanip>
#include <cstring>  // 用于 memcmp

// 构造函数：打开设备、初始化状态和防抖时间戳
JoyInterface::JoyInterface(const std::string& device) {
    running=true;
    joy_fd_ = -1;
    is_connected_ = false;
    InitJoyState();          // 初始化当前状态
    last_state_ = current_state_;  // 缓存初始状态

    // 打开设备（O_RDONLY：只读；O_NONBLOCK：非阻塞读取）
    joy_fd_ = open(device.c_str(), O_RDONLY | O_NONBLOCK);
    if (joy_fd_ < 0) {
        std::cerr << "[JoyInterface] 打开手柄设备失败！设备节点：" << device 
                  << "，错误信息：" << strerror(errno) << std::endl;
        return;
    }

    // 获取轴数量和按键数量（验证设备有效性）
    int axes_count = 0, buttons_count = 0;
    if (ioctl(joy_fd_, JSIOCGAXES, &axes_count) == -1) {
        std::cerr << "[JoyInterface] 获取轴数量失败：" << strerror(errno) << std::endl;
    } else {
        std::cout << "[JoyInterface] 轴数量：" << axes_count << std::endl;
    }

    if (ioctl(joy_fd_, JSIOCGBUTTONS, &buttons_count) == -1) {
        std::cerr << "[JoyInterface] 获取按键数量失败：" << strerror(errno) << std::endl;
    } else {
        std::cout << "[JoyInterface] 按键数量：" << buttons_count << std::endl;
    }

    // 初始化按键防抖时间戳（所有按键初始化为当前时间）
    for (int i = 0; i < 16; ++i) {
        button_last_time_[i] = std::chrono::steady_clock::now();
    }

    is_connected_ = true;
    std::cout << "[JoyInterface] 成功连接手柄！设备节点：" << device << std::endl;

    // 启动读取线程
    read_thread = std::thread(&JoyInterface::read_thread_, this);
}

// 析构函数：关闭设备，释放资源
JoyInterface::~JoyInterface() {
    if (is_connected_ && joy_fd_ >= 0) {
        close(joy_fd_);
        joy_fd_ = -1;
        is_connected_ = false;
        std::cout << "[JoyInterface] 手柄设备已断开连接" << std::endl;
    }
    running=false;
    // 等待读取线程结束
    if (read_thread.joinable()) {
        read_thread.join();
    }
}

// 私有工具函数：初始化手柄状态（所有按键松开，摇杆中立）
void JoyInterface::InitJoyState() {
    current_state_.A = 0;
    current_state_.B = 0;
    current_state_.X = 0;
    current_state_.Y = 0;
    current_state_.LB = 0;
    current_state_.RB = 0;
    current_state_.UP = 0;
    current_state_.DOWN = 0;
    current_state_.LEFT = 0;
    current_state_.RIGHT = 0;
    current_state_.LS[0] = 0.0;
    current_state_.LS[1] = 0.0;
    current_state_.RS[0] = 0.0;
    current_state_.RS[1] = 0.0;
}

// 私有工具函数：死区处理（输入原始值，输出归一化后的值）
double JoyInterface::ApplyDeadzone(int raw_value) {
    // 1. 原始值归一化到 [-1.0, 1.0]
    double normalized = static_cast<double>(raw_value) / STICK_RAW_MAX;
    // 2. 死区过滤：绝对值小于死区阈值则视为0
    if (std::fabs(normalized) < DEADZONE_RATIO) {
        return 0.0;
    }
    // 3. 死区外线性映射（消除死区导致的非线性）
    return (normalized - std::copysign(DEADZONE_RATIO, normalized)) / (1.0 - DEADZONE_RATIO);
}

// 私有工具函数：按键防抖校验（返回true表示状态稳定，可更新）
bool JoyInterface::DebounceCheck(int button_index) {
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - button_last_time_[button_index]
    );
    if (duration.count() > DEBOUNCE_DELAY_MS) {
        button_last_time_[button_index] = now;
        return true;
    }
    return false;
}


bool JoyInterface::IsStateChanged() {
    return std::memcmp(&current_state_, &last_state_, sizeof(JoyState)) != 0;
}


void JoyInterface::read_thread_()
{
    while (running)
    {
        ReadAndProcessEvents();
    }
    
}
// 公共接口：批量读取并处理所有待处理事件（核心逻辑）
void JoyInterface::ReadAndProcessEvents() {
    if (!is_connected_) {
        std::cerr << "[JoyInterface] 手柄未连接，无法读取数据" << std::endl;
        return;
    }

    // 循环读取缓冲区中所有未处理的事件（批量处理，不遗漏多按键）
    while (true) {
        ssize_t read_bytes = read(joy_fd_, &raw_event_, sizeof(js_event));
        if (read_bytes < 0) {
            if (errno == EAGAIN) {
                break;// 缓冲区无更多事件，退出循环
            } else {
                // std::cerr << "[JoyInterface] 读取事件失败：" << strerror(errno) << std::endl;
                break;   
            }
        }

        if (read_bytes != sizeof(js_event)) {
            break;  // 事件数据不完整，跳过
        }

        // 过滤初始化事件（设备连接时的初始状态，无需处理）
        uint8_t event_type = raw_event_.type & ~JS_EVENT_INIT;

        // 1. 处理按键事件（JS_EVENT_BUTTON = 0x01）
        if (event_type == JS_EVENT_BUTTON) {
            if (DebounceCheck(raw_event_.number)) {
                switch (raw_event_.number) {
                    case 0: current_state_.A = raw_event_.value; break;
                    case 1: current_state_.B = raw_event_.value; break;
                    case 2: current_state_.X = raw_event_.value; break;
                    case 3: current_state_.Y = raw_event_.value; break;
                    case 4: current_state_.LB = raw_event_.value; break;
                    case 5: current_state_.RB = raw_event_.value; break;
                    default:
                        // 打印未映射的按键编码，方便用户调整映射
                        std::cout << "[未映射按键] 编码：" << raw_event_.number 
                                  << " 状态：" << (raw_event_.value ? "按下" : "松开") << std::endl;
                        break;
                }
            }
        }

        // 2. 处理轴事件（JS_EVENT_AXIS = 0x02）
        else if (event_type == JS_EVENT_AXIS) {
            switch (raw_event_.number) {
                // 左摇杆X轴（0：左=-1，右=1）
                case 0:
                    current_state_.LS[0] = std::round(ApplyDeadzone(raw_event_.value) * 100) / 100.0;
                    break;
                // 左摇杆Y轴（1：上=1，下=-1，原始值上为负，取反）
                case 1:
                    current_state_.LS[1] = std::round(-ApplyDeadzone(raw_event_.value) * 100) / 100.0;
                    break;
                // 右摇杆X轴（3：左=-1，右=1，部分手柄为2，需根据实际调整）
                case 3:
                    current_state_.RS[0] = std::round(ApplyDeadzone(raw_event_.value) * 100) / 100.0;
                    break;
                // 右摇杆Y轴（4：上=1，下=-1，原始值上为负，取反）
                case 4:
                    current_state_.RS[1] = std::round(-ApplyDeadzone(raw_event_.value) * 100) / 100.0;
                    break;
                // 十字键X轴（6：左=-1，右=1，0=中立）
                case 6:
                    if (DebounceCheck(6)) {
                        current_state_.LEFT = (raw_event_.value <0) ? 1 : 0;
                        current_state_.RIGHT = (raw_event_.value >0) ? 1 : 0;
                    }
                    break;
                // 十字键Y轴（7：上=-1，下=1，0=中立）
                case 7:
                    if (DebounceCheck(7)) {
                        current_state_.UP = (raw_event_.value <0) ? 1 : 0;
                        current_state_.DOWN = (raw_event_.value >0) ? 1 : 0;
                    }
                    break;
                default:
                //     // 打印未映射的轴编码，方便用户调整映射
                //     std::cout << "[未映射轴] 编码：" << raw_event_.number 
                //               << " 数值：" << raw_event_.value << std::endl;
                    break;
            }
        }

        // 同步事件（JS_EVENT_SYN = 0x03）无需处理，仅关注按键/轴事件
    }

    // 更新上一次状态（供下次判断是否变化）
    last_state_ = current_state_;
}

// 公共接口：格式化打印当前手柄状态（无清屏，由外部控制）
void JoyInterface::PrintState() const {
    std::cout << "==================================== 手柄状态 ====================================" << std::endl;
    std::cout << "功能按键：" << std::endl;
    std::cout << "A键:" << (current_state_.A ? "\033[32m按下\033[0m" : "松开") << "\t";
    std::cout << "B键:" << (current_state_.B ? "\033[32m按下\033[0m" : "松开") << "\t";
    std::cout << "X键:" << (current_state_.X ? "\033[32m按下\033[0m" : "松开") << "\t";
    std::cout << "Y键:" << (current_state_.Y ? "\033[32m按下\033[0m" : "松开") << std::endl;

    std::cout << "十字键：" << std::endl;
    std::cout << "上键：" << (current_state_.UP ? "\033[32m按下\033[0m" : "松开") << "\t";
    std::cout << "下键：" << (current_state_.DOWN ? "\033[32m按下\033[0m" : "松开") << "\t";
    std::cout << "左键：" << (current_state_.LEFT ? "\033[32m按下\033[0m" : "松开") << "\t";
    std::cout << "右键：" << (current_state_.RIGHT ? "\033[32m按下\033[0m" : "松开") << "\t" << std::endl;

    std::cout << "肩键：" << std::endl;
    std::cout << "LB键:" << (current_state_.LB ? "\033[32m按下\033[0m" : "松开") << "\t";
    std::cout << "RB键:" << (current_state_.RB ? "\033[32m按下\033[0m" : "松开") << std::endl;

    std::cout << "摇杆(归一化值，-1.0~1.0):" << std::endl;
    std::cout << "左摇杆 X轴:" << std::fixed << std::setprecision(2) << current_state_.LS[0] << "\t";
    std::cout << "左摇杆 Y轴:" << std::fixed << std::setprecision(2) << current_state_.LS[1] << std::endl;
    std::cout << "右摇杆 X轴:" << std::fixed << std::setprecision(2) << current_state_.RS[0] << "\t";
    std::cout << "右摇杆 Y轴:" << std::fixed << std::setprecision(2) << current_state_.RS[1] << std::endl;
    std::cout << "=================================================================================" << std::endl;
    std::cout << "提示：按 Ctrl+C 退出程序" << std::endl;
}

// 公共接口：获取当前手柄状态（只读，外部程序可调用）
const JoyState& JoyInterface::GetCurrentState() const {
    return current_state_;
}

// 公共接口：判断手柄是否连接
bool JoyInterface::IsConnected() const {
    return is_connected_;
}
