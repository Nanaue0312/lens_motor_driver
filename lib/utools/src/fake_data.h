#ifndef __FAKE_DATA_HPP__
#define __FAKE_DATA_HPP__

#include <string>
#include <random>
#include <chrono>
#include <type_traits>
#include <stdexcept>
#include <algorithm>
#include <numeric>

namespace utools::fake_data
{
    /// @brief 生成指定范围的随机整数
    /// @param min 最大值
    /// @param max 最大值
    /// @return 随机数
    template <typename ValTyp>
    ValTyp rand_impl(ValTyp min, ValTyp max, std::true_type)
    {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_int_distribution<ValTyp> dis(min, max);
        return dis(gen);
    }

    template <typename ValTyp>
    ValTyp rand_impl(ValTyp min, ValTyp max, std::false_type)
    {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_real_distribution<ValTyp> dis(min, max);
        return dis(gen);
    }

    /// @brief 生成指定范围的随机整数
    /// @param min 最大值
    /// @param max 最大值
    /// @return 随机数
    template <typename ValTyp,
              typename = std::enable_if<std::is_integral<ValTyp>::value ||
                                        std::is_floating_point<ValTyp>::value>>
    ValTyp rand(ValTyp min, ValTyp max)
    {
        return rand_impl(min, max, std::is_integral<ValTyp>());
    }

    template <typename ValTyp>
    ValTyp rand(ValTyp max = std::is_integral<ValTyp>::value ? 100 : 1.0)
    {
        return rand(static_cast<ValTyp>(0), max);
    }

    /// @brief 生成指定长度的随机字符串
    /// @tparam _PRINT_STR 是否打印字符串
    /// @param len 字符串长度
    /// @return 随机字符串
    template <bool _PRINT_STR = true>
    std::string rand_str(const size_t len)
    {
        std::string result;
        result.reserve(len);
        if (_PRINT_STR)
        {
            static const char alphanum[] =
                "0123456789"
                "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                "abcdefghijklmnopqrstuvwxyz"; // 数字和字母字符集
            for (size_t i = 0; i < len; ++i)
            {
                result += alphanum[rand<int>(0, sizeof(alphanum) - 1)];
            }
        }
        else
        {
            for (size_t i = 0; i < len; ++i)
            {
                result += static_cast<char>(rand<char>(32, 126)); // 所有可打印字符集
            }
        }
        return result;
    }

    /// @brief 生成在指定长度范围内的字符串
    /// @tparam _PRINT_STR 是否打印字符串
    /// @param min_len 最小长度
    /// @param max_len 最大长度
    /// @return 随机字符串
    template <bool _PRINT_STR = true>
    std::string rand_str(const size_t min_len, const size_t max_len)
    {
        return rand_str<_PRINT_STR>(rand(min_len, max_len));
    }
}

#endif // __FAKE_DATA_HPP__
