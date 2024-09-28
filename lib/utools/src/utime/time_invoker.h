#ifndef __TIMED_INVOKER_H__
#define __TIMED_INVOKER_H__

#include <chrono>
#include <cstdint>
#include <functional>
#include <utility>

/// @brief 非线程安全的计时器
namespace utools::time
{
    struct timed_invoker_result_t
    {
    public:
        template <typename _TimeType = std::chrono::milliseconds>
        const int64_t elapsed() const
        {
            if (__is_invoked_count > 0 && !__is_invoking)
            {
                return std::chrono::duration_cast<_TimeType>(__end_time - __start_time).count();
            }
            if (__is_invoking)
            {
                return std::chrono::duration_cast<_TimeType>(std::chrono::high_resolution_clock::now() - __start_time).count();
            }
            return 0;
        }

        const int32_t count() const
        {
            return __is_invoked_count;
        }

        const int64_t total_elapsed() const
        {
            return __total_invoked_time_ms;
        }

        const int64_t average_elapsed() const
        {
            if (__is_invoked_count > 0)
            {
                return __total_invoked_time_ms / __is_invoked_count;
            }
            return 0;
        }

        const int64_t max_elapsed() const
        {
            return __max_invoked_time_ms;
        }

        const int64_t min_elapsed() const
        {
            return __min_invoked_time_ms;
        }

        bool reset()
        {
            if (__is_invoking)
            {
                return false;
            }
            __is_invoked_count = 0;
            __total_invoked_time_ms = 0;
            __max_invoked_time_ms = std::numeric_limits<int64_t>::min();
            __min_invoked_time_ms = std::numeric_limits<int64_t>::max();
            return true;
        }

    protected:
        std::chrono::high_resolution_clock::time_point __start_time;
        std::chrono::high_resolution_clock::time_point __end_time;
        int32_t __is_invoked_count{0};
        bool __is_invoking{false};
        int64_t __total_invoked_time_ms{0};
        int64_t __max_invoked_time_ms{std::numeric_limits<int64_t>::min()};
        int64_t __min_invoked_time_ms{std::numeric_limits<int64_t>::max()};

        bool __update()
        {
            if (!__is_invoking)
            {
                return false;
            }
            // 计算本次调用所用的时间
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(__end_time - __start_time).count();
            // 更新最大和最小执行时间
            if (duration > __max_invoked_time_ms)
            {
                __max_invoked_time_ms = duration;
            }
            if (duration < __min_invoked_time_ms)
            {
                __min_invoked_time_ms = duration;
            }
            // 更新总调用次数和总时长
            ++__is_invoked_count;
            __total_invoked_time_ms += duration;
            return true;
        }
    };

    /// @brief 计算函数执行时间
    template <typename Func>
    class TimedInvoker : public timed_invoker_result_t
    {
    public:
        TimedInvoker(Func &&func) : func_(std::forward<Func>(func)) {}

        template <typename... Args>
        auto invoke(Args &&...args) -> decltype(std::declval<Func>()(std::forward<Args>(args)...))
        {
            using namespace std::chrono;
            __is_invoking = true;
            // 运行任务
            __start_time = high_resolution_clock::now();
            auto result = func_(std::forward<Args>(args)...);
            __end_time = high_resolution_clock::now();
            __update(); // 更新统计信息
            __is_invoking = false;
            return result;
        }

        template <typename... Args>
        auto operator()(Args &&...args) -> decltype(invoke(std::forward<Args>(args)...))
        {
            return invoke(std::forward<Args>(args)...);
        }

    private:
        Func func_;
    };

    /// @brief TimedInvoker的nullptr_t特化版本
    template <>
    class TimedInvoker<std::nullptr_t> : public timed_invoker_result_t
    {
    public:
        TimedInvoker(std::nullptr_t) {}

        /// @brief 再重新开始一次
        /// @return 如果正在运行，不可以再次调用，返回false
        bool start()
        {
            if (__is_invoking)
            {
                return false;
            }
            __is_invoking = true;
            __start_time = std::chrono::high_resolution_clock::now();
            return true;
        }

        bool stop()
        {
            if (!__is_invoking)
            {
                return false;
            }
            __end_time = std::chrono::high_resolution_clock::now();
            __update(); // 更新统计信息
            __is_invoking = false;
            return true; // 返回一个默认值，因为空函数没有实际的返回值
        }
    };

    /// @brief 处理类成员函数的定时统计辅助函数
    /// @tparam ReturnType 返回值类型
    /// @tparam T 类指针
    /// @tparam ...Args 参数类型
    /// @param func 成员函数指针
    /// @param obj 类指针
    /// @return 时间统计对象
    template <typename ReturnType, typename T, typename... Args>
    auto make_timed_invoker(ReturnType (T::*func)(Args...), T *obj)
        -> TimedInvoker<std::function<ReturnType(Args...)>>
    {
        auto func_bound = [func, obj](Args... args) -> ReturnType
        {
            return (obj->*func)(std::forward<Args>(args)...);
        };
        return TimedInvoker<std::function<ReturnType(Args...)>>(func_bound);
    }

    /// @brief 处理非成员函数的定时统计辅助函数
    /// @tparam ReturnType 返回值类型
    /// @tparam ...Args 参数类型
    /// @param func 函数指针
    /// @return 时间统计对象
    template <typename ReturnType, typename... Args>
    auto make_timed_invoker(ReturnType (*func)(Args &&...))
        -> TimedInvoker<decltype(func)>
    {
        return TimedInvoker<decltype(func)>(func);
    }

    /// @brief 辅助函数来简化类的使用
    /// @tparam Func 函数类型
    /// @param func 函数，默认为空
    template <typename Func>
    auto make_timed_invoker(Func &&func) -> TimedInvoker<Func>
    {
        return TimedInvoker<Func>(std::forward<Func>(func));
    }

    /// @brief nullptr_t特化版本的辅助函数
    inline auto make_timed_invoker(std::nullptr_t func) -> TimedInvoker<std::nullptr_t>
    {
        return TimedInvoker<std::nullptr_t>(func);
    }
}

#endif // __TIMED_INVOKER_H__
