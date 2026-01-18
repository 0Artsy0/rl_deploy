#pragma once
#include <thread>
#include <chrono>
#include <functional>
#include <atomic>
#include <string>
#include <memory>
#include <iostream>

class thread_Timer
{
private:
    std::unique_ptr<std::thread> _thread;

    std::atomic<bool> _running{false};
    std::atomic<bool> _stop_requested{false};

    double _frequency; // Hz
    
    std::function<void()> _callback;
    std::string _thread_name;
    
    void timer_thread_func()
    {
        auto interval_ms = std::chrono::duration<double, std::milli>(1000.0 / _frequency);
        
        while (!_stop_requested.load()) {
            auto start_time = std::chrono::steady_clock::now();
            
            try {
                // 执行回调函数
                if (_callback) {
                    _callback();
                }
            } catch (const std::exception& e) {
                std::cerr << "Exception in timer callback: " << e.what() << std::endl;
            } catch (...) {
                std::cerr << "Unknown exception in timer callback" << std::endl;
            }
            
            auto end_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration<double, std::milli>(end_time - start_time);
            auto sleep_time = interval_ms - elapsed;
            
            if (sleep_time > std::chrono::duration<double, std::milli>::zero()) {
                std::this_thread::sleep_for(sleep_time);
            }
        }
        _running.store(false);
    }

public:
    thread_Timer() = default;
    thread_Timer(double frequency, std::function<void()> callback, const std::string& thread_name = "")
        : _frequency(frequency), _callback(callback), _thread_name(thread_name)
    {
        start();
    }

    ~thread_Timer()
    {
        stop();
    }

    bool start(double frequency, std::function<void()> callback, const std::string& thread_name = "")
    {
        if (_running.load()) {
            std::cout << "Timer is already running" << std::endl;
            return false;
        }
        
        _frequency = frequency;
        _callback = callback;
        _thread_name = thread_name;
        
        return start();
    }

    bool start()
    {
        if (_running.load()) {
            std::cout << "Timer is already running" << std::endl;
            return false;
        }
        
        if (_frequency <= 0) {
            std::cerr << "Invalid frequency: " << _frequency << " Hz" << std::endl;
            return false;
        }
        
        if (!_callback) {
            std::cerr << "No callback function set" << std::endl;
            return false;
        }
        
        _stop_requested.store(false);
        _running.store(true);
        
        _thread = std::make_unique<std::thread>(&thread_Timer::timer_thread_func, this);
        
        return true;
    }

    void stop()
    {
        if (!_running.load()) {
            return;
        }
        
        _stop_requested.store(true);
        
        if (_thread && _thread->joinable()) {
            _thread->join();
        }
        
        _thread.reset();
        _running.store(false);
    }

    // 禁用拷贝构造和赋值
    thread_Timer(const thread_Timer&) = delete;
    thread_Timer& operator=(const thread_Timer&) = delete;
    
    // 允许移动构造和赋值
    thread_Timer(thread_Timer&&) = default;
    thread_Timer& operator=(thread_Timer&&) = default;
};