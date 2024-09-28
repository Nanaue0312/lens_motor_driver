/// @brief 自旋等待，直到条件满足或超时

#ifndef __UTOLLS_SPIN_WAIT_H__
#define __UTOLLS_SPIN_WAIT_H__

#if UTOOLS_COLLAB_SPIN_WAIT_ENABLE == 1

#include <chrono>
#include <functional>
#include <thread>

namespace utools::collab
{
    /// @brief 自旋等待，直到条件满足或超时
    /// @tparam FUN 函数类型
    /// @tparam ...ARGS 参数类型
    /// @tparam Duration 等待时长
    /// @param fun 需要调用的函数
    /// @param timeout 超时时长
    /// @param ...args 函数参数
    /// @return true/false 是否超时
    template <typename Duration, typename FUN, typename... ARGS>
    bool spin_wait_for(FUN &&fun, Duration timeout, ARGS &&...args)
    {
        auto startTime = std::chrono::steady_clock::now(); // 开始时间
        while (!fun(std::forward<ARGS>(args)...))
        {
            if (std::chrono::steady_clock::now() - startTime >= timeout)
            {
                return false; // 超时，返回false
            }
            std::this_thread::yield(); // 让出CPU时间片，以降低CPU占用
        }
        return true; // 条件满足，返回true
    }

    /// @brief 自旋等待，直到条件满足或到达指定时间
    /// @tparam FUN 函数类型
    /// @tparam ...ARGS 参数类型
    /// @tparam Duration 等待时长
    /// @param fun 需要调用的函数
    /// @param timeout 超时时长
    /// @param ...args 函数参数
    template <typename FUN, typename... ARGS>
    bool spin_wait_until(FUN &&fun, std::chrono::steady_clock::time_point timeout, ARGS &&...args)
    {
        while (!fun(std::forward<ARGS>(args)...))
        {
            if (std::chrono::steady_clock::now() >= timeout)
            {
                return false; // 超时，返回false
            }
            std::this_thread::yield(); // 让出CPU时间片，以降低CPU占用
        }
        return true; // 条件满足，返回true
    }
}

#endif // UTOOLS_COLLAB_SPIN_WAIT_ENABLE
#endif // __UTOLLS_SPIN_WAIT_H__
