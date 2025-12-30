#ifndef JOY_INTERFACE_HPP
#define JOY_INTERFACE_HPP

#include <string>
#include <chrono>
#include <cstring>
#include <linux/joystick.h>
#include <thread>

// 手柄设备默认节点（可在初始化时修改）
#define DEFAULT_JOY_PORT "/dev/input/js0"
// 摇杆原始最大值（16位有符号数，大多数手柄通用）
#define STICK_RAW_MAX 32767
// 死区比例（3%，消除中立位置漂移）
#define DEADZONE_RATIO 0.03
// 按键防抖延时（10ms，避免机械抖动误触发）
#define DEBOUNCE_DELAY_MS 10
// 定时打印间隔（50ms，平衡实时性和视觉效果）
#define PRINT_INTERVAL_MS 50

// 手柄状态结构体：存储所有按键和轴的当前状态
typedef struct
{
    // 功能按键（0=松开，1=按下）
    int A;
    int B;
    int X;
    int Y;
    int LB;
    int RB;
    // 十字键（0=松开，1=按下）
    int UP;
    int DOWN;
    int LEFT;
    int RIGHT;
    // 摇杆（归一化到 [-1.0, 1.0]，0.0=中立）
    double LS[2]; // 左摇杆：[X轴, Y轴]（X：左=-1，右=1；Y：上=1，下=-1）
    double RS[2]; // 右摇杆：[X轴, Y轴]
} JoyState;

// 手柄接口类：封装设备打开、批量事件处理、状态缓存
class JoyInterface
{
private:
    int joy_fd_;         // 手柄设备文件描述符
    bool is_connected_;  // 手柄是否连接成功
    js_event raw_event_; // 读取的原始事件
    // 按键防抖时间记录（每个按键对应一个时间戳）
    std::chrono::time_point<std::chrono::steady_clock> button_last_time_[16];

    // 私有工具函数：应用死区处理（消除摇杆漂移）
    double ApplyDeadzone(int raw_value);
    // 私有工具函数：按键防抖校验（避免机械抖动）
    bool DebounceCheck(int button_index);
    // 私有工具函数：初始化手柄状态（所有值设为默认）
    void InitJoyState();

    bool running=false; // 运行标志位，用于控制循环运行
    std::thread read_thread; // 读取线程，用于异步处理事件

public:
    // 构造函数：传入手柄设备节点（默认使用 DEFAULT_JOY_PORT）
    JoyInterface(const std::string &device = DEFAULT_JOY_PORT);
    // 析构函数：关闭设备
    ~JoyInterface();

    // 公共接口：批量读取并处理所有待处理事件（无返回值，内部缓存状态）
    void ReadAndProcessEvents();
    // 公共接口：获取当前手柄状态（只读，外部可调用）
    const JoyState &GetCurrentState() const;
    // 公共接口：打印当前状态（格式化输出）
    void PrintState() const;
    // 公共接口：判断手柄是否连接
    bool IsConnected() const;
    // 公共接口：判断状态是否变化（修改为public，允许外部调用）
    bool IsStateChanged();

    void read_thread_();

    JoyState current_state_; // 当前手柄状态
    JoyState last_state_;    // 上一次状态（用于判断是否变化）
};

#endif // JOY_INTERFACE_HPP