/// @brief 稳定时间调用，控制周期之间的调用时间间隔

#ifndef __UTOOLS_STABLE_INTERVAL_INVOKER_H__
#define __UTOOLS_STABLE_INTERVAL_INVOKER_H__

#if UTOOLS_STABLE_INTERVAL_INVOKER_ENABLE == 1

#include <stdint.h>
#include <chrono>
#include <tuple>
#include <functional>
#include <thread>

namespace utools::time
{
    template <typename Func>
    class StableIntervalInvoker
    {
    private:
        Func __func;

        bool __running{false}; // 运行状态

        // 默认间隔1秒
        std::chrono::milliseconds __interval{std::chrono::seconds(1)};

    public:
        using stable_interval_invoker_t = StableIntervalInvoker<Func>;
        using type = StableIntervalInvoker<Func>;
        using func_t = Func;

        /// @brief 调用间隔
        struct invoker_sleep_t
        {
        private:
            std::chrono::high_resolution_clock::time_point __end_time;

        public:
            invoker_sleep_t() = delete;
            virtual ~invoker_sleep_t() = default;

            invoker_sleep_t(std::chrono::high_resolution_clock::time_point end_time) : __end_time(end_time) {};

            void operator()()
            {
                sleep();
            }

            inline void sleep()
            {
                utools::time::sleep_until(__end_time);
            }
        };

        StableIntervalInvoker(Func &&func) : __func(std::forward<Func>(func)) {}

        StableIntervalInvoker(int64_t interval, Func &&func)
            : __func(std::forward<Func>(func)), __interval(std::chrono::milliseconds(interval)) {}

        StableIntervalInvoker(const int64_t &interval)
            : __interval(std::chrono::milliseconds(interval)) {}

        virtual ~StableIntervalInvoker() = default;

        /// @brief 设置运行任务
        /// @param func 可调用对象
        /// @param args 可变参数
        /// @return 返回任务对象本身
        template <typename Func_, typename... Args>
        stable_interval_invoker_t &bind(Func_ &&func, Args &&...args)
        {
            __func = std::bind(std::forward<Func_>(func), std::forward<Args>(args)...);
            return *this;
        }

        /// @brief 设置运行任务
        /// @param func 类成员函数
        /// @param obj 类对象指针
        /// @param args 可变参数
        /// @return 返回任务对象本身
        template <typename ReturnType, typename T, typename... Args>
        stable_interval_invoker_t &bind(ReturnType (T::*func)(Args...), T *obj, Args &&...args)
        {
            __func = std::bind(func, obj, std::forward<Args>(args)...);
            return *this;
        }

        /// @brief 设置运行任务
        /// @param func 静态函数或自由函数
        /// @param args 可变参数
        /// @return 返回任务对象本身
        template <typename ReturnType, typename... Args>
        stable_interval_invoker_t &bind(ReturnType (*func)(Args...), Args &&...args)
        {
            __func = std::bind(func, std::forward<Args>(args)...);
            return *this;
        }

        /// @brief 调用
        /// @return tuple<result, invoker_sleep_t对象>，invoker_sleep_t对象可以通过invoker_sleep_t()的方式调用，产生延时
        template <typename... Args>
        auto invoke(Args &&...args)
            -> typename std::conditional<
                std::is_void<decltype(std::declval<Func>()(std::forward<Args>(args)...))>::type,
                invoker_sleep_t,
                std::tuple<decltype(std::declval<Func>()(std::forward<Args>(args)...)), invoker_sleep_t>>::type
        {
            auto end_time = std::chrono::high_resolution_clock::now() + __interval;

            using ReturnType = decltype(std::declval<Func>()(std::forward<Args>(args)...));

            if constexpr (std::is_void<ReturnType>::type)
            {
                __func(std::forward<Args>(args)...);
                return invoker_sleep_t(end_time);
            }
            else
            {
                auto result = __func(std::forward<Args>(args)...);
                return {result, invoker_sleep_t{end_time}};
            }
        }

        /// @brief 调用
        template <typename... Args>
        auto operator()(Args &&...args) -> decltype(invoke(std::forward<Args>(args)...))
        {
            return invoke(std::forward<Args>(args)...);
        }

        /// @brief 调用，此函数会自动等待，并延时返回调用执行的结果（如果有的话）
        /// @return 返回结果
        template <typename... Args>
        auto invoke_defer(Args &&...args)
            -> decltype(std::declval<Func>()(std::forward<Args>(args)...))
        {
            auto end_time = std::chrono::high_resolution_clock::now() + __interval;

            using ReturnType = decltype(std::declval<Func>()(std::forward<Args>(args)...));

            if constexpr (std::is_same<ReturnType, void>::value)
            {
                __func(std::forward<Args>(args)...);
                utools::time::sleep_until(end_time);
            }
            else
            {
                auto result = __func(std::forward<Args>(args)...);
                utools::time::sleep_until(end_time);
                return result;
            }
        }

        /// @brief 设置调用间隔
        /// @param interval_ms 调用间隔，单位毫秒
        /// @return 返回自身
        stable_interval_invoker_t &set_invoker_interval(const int64_t &interval_ms)
        {
            __interval = std::chrono::milliseconds(interval_ms);
            return *this;
        }

        /// @brief 获取调用间隔
        /// @return 调用间隔
        /// @note 单位为毫秒
        const int64_t get_invoker_interval() const
        {
            return __interval.count();
        }

        /// @brief 一直周期性调用，直到__running为false，此方式调用，可以传入参数，但不会返回数据
        template <typename... Args>
        void run_forever(Args &&...args)
        {
            if (__running)
            {
                return;
            }

            __running = true;
            while (__running)
            {
                auto end_time = std::chrono::high_resolution_clock::now() + __interval;
                __func(std::forward<Args>(args)...);
                utools::time::sleep_until(end_time);
            }
        }

        /// @brief 如果调用了run_forever，则调用stop()停止调用
        void stop()
        {
            __running = false;
        }

        /// @brief 是否正在运行
        /// @return true表示正在运行，false表示停止
        bool is_running() const
        {
            return __running;
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
    auto make_stable_interval_invoker(ReturnType (T::*func)(Args...), T *obj)
        -> StableIntervalInvoker<std::function<ReturnType(Args...)>>
    {
        auto func_bound = [func, obj](Args... args) -> ReturnType
        {
            return (obj->*func)(std::forward<Args>(args)...);
        };
        return StableIntervalInvoker<std::function<ReturnType(Args...)>>(func_bound);
    }

    /// @brief 处理非成员函数的定时统计辅助函数
    /// @tparam ReturnType 返回值类型
    /// @tparam ...Args 参数类型
    /// @param func 函数指针
    /// @return 时间统计对象
    template <typename ReturnType, typename... Args>
    auto make_stable_interval_invoker(ReturnType (*func)(Args &&...))
        -> StableIntervalInvoker<decltype(func)>
    {
        return StableIntervalInvoker<decltype(func)>(func);
    }

    /// @brief 辅助函数来简化类的使用
    /// @tparam Func 函数类型
    /// @param func 函数，默认为空
    template <typename Func>
    auto make_stable_interval_invoker(Func &&func) -> StableIntervalInvoker<Func>
    {
        return StableIntervalInvoker<Func>(std::forward<Func>(func));
    }

} // namespace utools::time

#endif // UTOOLS_STABLE_INTERVAL_INVOKER_ENABLE
#endif // __UTOOLS_STABLE_INTERVAL_INVOKER_H__
