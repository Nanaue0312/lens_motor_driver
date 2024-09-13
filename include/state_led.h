#ifndef __STATE_LED_H__
#define __STATE_LED_H__

#include <functional>

/// @brief 状态灯管理类，处理状态灯的亮灭
class StateLED
{
private:
    std::function<void()> __on_func;
    std::function<void()> __off_func;
    bool __is_on;

public:
    StateLED() = delete;

    /// @brief 构造函数
    /// @param on_func 开灯函数
    /// @param off_func 关灯函数
    /// @param def_on 默认是否开灯
    StateLED(std::function<void()> on_func, std::function<void()> off_func, bool def_on = false)
        : __on_func(on_func), __off_func(off_func)
    {
        if (def_on)
        {
            on();
        }
        else
        {
            off();
        }
    }

    virtual ~StateLED() = default;

    /// @brief 开灯
    void on()
    {
        __on_func();
        __is_on = true;
    }

    /// @brief 关灯
    void off()
    {
        __off_func();
        __is_on = false;
    }

    /// @brief 切换灯的状态
    void toggle()
    {
        if (__is_on)
        {
            off();
        }
        else
        {
            on();
        }
    }

    bool is_on()
    {
        return __is_on;
    }
};

#endif // __STATE_LED_H__