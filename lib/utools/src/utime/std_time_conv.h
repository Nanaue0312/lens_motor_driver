#ifndef __STD_TIME_CONV_H__
#define __STD_TIME_CONV_H__

#include <chrono>
#include <type_traits>
#include <cstdint>

namespace utools::time
{
    // 辅助类型萃取，用于区分时间点和整数
    template <typename T>
    struct is_time_point : std::false_type
    {
    };

    template <typename Clock, typename Duration>
    struct is_time_point<std::chrono::time_point<Clock, Duration>> : std::true_type
    {
    };

    // 专门为std::chrono::time_point类型实现的函数
    template <typename Duration, typename Clock, typename TimePointDuration>
    Duration to_duration_impl(const std::chrono::time_point<Clock, TimePointDuration> &tp)
    {
        return std::chrono::duration_cast<Duration>(tp.time_since_epoch());
    }

    // 专门为整数类型实现的函数
    template <typename Duration, typename IntType>
    typename std::enable_if<std::is_integral<IntType>::value, Duration>::type
    to_duration_impl(IntType milliseconds)
    {
        return std::chrono::duration_cast<Duration>(std::chrono::milliseconds(milliseconds));
    }

    /// @brief 将时间点或整数转换为指定的时间间隔类型
    /// @tparam Duration 目标时间间隔类型，为std::chrono::duration的派生类
    /// @tparam TP_OR_INT 传入的时间点或整数类型
    /// @param tp 输入数据
    /// @return 目标类型的时间间隔
    template <typename Duration, typename TP_OR_INT>
    Duration to_duration(const TP_OR_INT &tp)
    {
        if constexpr (is_time_point<TP_OR_INT>::value)
        {
            return to_duration_impl<Duration>(tp);
        }
        else if constexpr (std::is_integral<TP_OR_INT>::value)
        {
            return to_duration_impl<Duration>(tp);
        }
        else
        {
            static_assert(std::is_integral<TP_OR_INT>::value || is_time_point<TP_OR_INT>::value,
                          "to_duration requires a std::chrono::time_point or an integral type.");
            return Duration::zero();
        }
    }

    /// @brief 将时间点或整数转换为std::chrono::nanoseconds类型的时间间隔
    /// @tparam TP_OR_INT 传入时间点或整数类型
    /// @param tp 时间点或整数
    /// @return ns间间隔
    template <typename TP_OR_INT>
    std::chrono::nanoseconds to_std_ns(const TP_OR_INT &tp)
    {
        return to_duration<std::chrono::nanoseconds>(tp);
    }

    /// @brief 将时间点或整数转换为std::chrono::microseconds类型的时间间隔
    /// @tparam TP_OR_INT 传入时间点或整数类型
    /// @param tp 时间点或整数
    /// @return us间间隔
    template <typename TP_OR_INT>
    std::chrono::microseconds to_std_us(const TP_OR_INT &tp)
    {
        return to_duration<std::chrono::microseconds>(tp);
    }

    /// @brief 将时间点或整数转换为std::chrono::milliseconds类型的时间间隔
    /// @tparam TP_OR_INT 传入时间点或整数类型
    /// @param tp 时间点或整数
    /// @return ms间间隔
    template <typename TP_OR_INT>
    std::chrono::milliseconds to_std_ms(const TP_OR_INT &tp)
    {
        return to_duration<std::chrono::milliseconds>(tp);
    }

    /// @brief 将时间点或整数转换为std::chrono::seconds类型的时间间隔
    /// @tparam TP_OR_INT 传入时间点或整数类型
    /// @param tp 时间点或整数
    /// @return s间间隔
    template <typename TP_OR_INT>
    std::chrono::seconds to_std_s(const TP_OR_INT &tp)
    {
        return to_duration<std::chrono::seconds>(tp);
    }

    /// @brief 将时间点或整数转换为std::chrono::minutes类型的时间间隔
    /// @tparam TP_OR_INT 传入时间点或整数类型
    /// @param tp 时间点或整数
    /// @return m间间隔
    template <typename TP_OR_INT>
    std::chrono::minutes to_std_m(const TP_OR_INT &tp)
    {
        return to_duration<std::chrono::minutes>(tp);
    }

    /// @brief 将时间点或整数转换为std::chrono::hours类型的时间间隔
    /// @tparam TP_OR_INT 传入时间点或整数类型
    /// @param tp 时间点或整数
    /// @return h间间隔
    template <typename TP_OR_INT>
    std::chrono::hours to_std_h(const TP_OR_INT &tp)
    {
        return to_duration<std::chrono::hours>(tp);
    }
}

#endif // __STD_TIME_CONV_H__