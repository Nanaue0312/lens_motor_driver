#ifndef __UTOOLS__STR_CONV_H__
#define __UTOOLS__STR_CONV_H__

#include <stdint.h>
#include <type_traits>
#include <string>

namespace utools::fmt
{
    /// @brief 将整数转换为字符串
    /// @tparam T 整数类型
    /// @param n 整数值
    /// @param base 进制
    /// @return 数字字符串
    template <typename T, typename = std::enable_if<std::is_integral<T>::value>>
    std::string int_to_string(T n, uint8_t base = 10)
    {
        char buf[8 * sizeof(T) + 1]; // 假设 8 位字符和零字节
        char *str = &buf[sizeof(buf) - 1];

        *str = '\0'; // 结束符
        bool isNegative = false;

        if (std::is_signed<T>::value && n < 0)
        {
            isNegative = true;
            n = -n; // 处理负数
        }

        do
        {
            T m = n;
            n /= base;
            char c = m - base * n;
            *--str = c + '0'; // 转换为字符
        } while (n);

        if (isNegative)
        {
            *--str = '-'; // 添加负号
        }

        return std::string(str);
    }

    /// @brief 将浮点型转换为字符串
    /// @tparam T 小数类型
    /// @param n 小数值
    /// @param precision 小数位数
    /// @return 数字字符串
    template <typename T, typename = std::enable_if<std::is_floating_point<T>::value>>
    std::string float_to_string(T n, int precision = 3)
    {
        char buf[32]; // 足够大的字符数组
        char *str = buf;

        // 处理负数
        if (n < 0)
        {
            *str++ = '-';
            n = -n;
        }

        // 整部份
        long long int_part = static_cast<long long>(n);
        n -= int_part;

        // 处理整部份
        char *int_str = const_cast<char *>(::utools::fmt::int_to_string<long long>(int_part, 10).c_str());
        while (*int_str)
        {
            *str++ = *int_str++;
        }

        // 处理小数部分
        if (precision > 0)
        {
            *str++ = '.';
            for (int i = 0; i < precision; ++i)
            {
                n *= 10;
                long long digit = static_cast<long long>(n);
                *str++ = digit + '0';
                n -= digit;
            }
        }

        *str = '\0'; // 结束符
        return std::string(buf);
    }

    /// @return 数字字符串
    template <typename T>
    std::string to_string(T n, int mode_or_precision = -1)
    {
        using DecayedT = typename std::decay<T>::type; // 去除引用和常量性

        if constexpr (std::is_integral<DecayedT>::value)
        {
            return int_to_string(n, mode_or_precision < 1 ? 10 : mode_or_precision);
        }
        else if constexpr (std::is_floating_point<DecayedT>::value)
        {
            return float_to_string(n, mode_or_precision < 1 ? 3 : mode_or_precision);
        }
        else if constexpr (std::is_same<DecayedT, char>::value)
        {
            return std::string(1, n);
        }
        else if constexpr (std::is_same<DecayedT, const char *>::value)
        {
            return std::string(n);
        }
        else if constexpr (std::is_same<DecayedT, std::string>::value)
        {
            return n;
        }
        else if constexpr (std::is_same<DecayedT, bool>::value)
        {
            return n ? "true" : "false";
        }
        else if constexpr (std::is_same<DecayedT, std::nullptr_t>::value)
        {
            return "nullptr";
        }
        else if constexpr (std::is_enum<DecayedT>::value)
        {
            return int_to_string(static_cast<int>(n), 10);
        }
        else if constexpr (std::is_pointer<DecayedT>::value)
        {
            return int_to_string(static_cast<uint32_t>(reinterpret_cast<uintptr_t>(n)), 10);
        }
        return "";
    }
} // namespace utools::fmt

#endif // __UTOOLS__STR_CONV_H__