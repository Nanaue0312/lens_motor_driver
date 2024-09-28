#ifndef __F_STRING_H__
#define __F_STRING_H__

/// @brief 在xil sdk中，str函数被定义为宏，这里需要先取消定义
#ifdef str
#define old_str_function(...) str(__VA_ARGS__)
#undef str
#endif

#include <sstream>
#include <string>
#include <cstdarg>

namespace utools::fmt
{
    /// @brief 当没有参数时，什么也不做并返回空字符串
    std::string f();

    /// @brief 使用 C++ 风格的格式化功能
    /// @param ... 可变参数
    /// @return 格式化后的字符串
    template <typename T, typename... Args>
    std::string f(const T &first, const Args &...args)
    {
        std::ostringstream ss;
        ss << first; // 处理当前参数
        if (sizeof...(args) > 0)
        {
            ss << " "; // 只有在还有剩余参数时才添加空格
        }
        ss << f(args...); // 递归处理剩余参数
        return ss.str();
    }

    /// @brief 使用 C++ 风格的格式化功能，并添加分隔符和结束字符串
    /// @param separator 分隔符
    /// @param endstr 结束字符串
    /// @return 格式化后的字符串
    std::string f_with_ctrl(const std::string &separator = "", const std::string &endstr = "");

    /// @brief 使用 C++ 风格的格式化功能，并添加分隔符和结束字符串
    /// @param separator 分隔符
    /// @param endstr 结束字符串
    /// @param ... 可变参数
    /// @return 格式化后的字符串
    template <typename T, typename... Args>
    std::string f_with_ctrl(const std::string &separator, const std::string &endstr, const T &first, const Args &...args)
    {
        std::ostringstream ss;

        if constexpr (std::is_same<T, uint8_t>::value || std::is_same<T, int8_t>::value)
        {
            ss << static_cast<int>(first); // 处理 uint8_t 和 int8_t 类型
        }
        else
        {
            ss << first;
        }

        if (sizeof...(args) > 0)
        {
            ss << separator;
        }

        ss << f_with_ctrl(separator, endstr, args...);

        if (sizeof...(args) == 0 && !endstr.empty())
        {
            ss << endstr;
        }

        return ss.str();
    }

    /// @brief 使用 C 风格的格式化功能，并添加分隔符和结束字符串
    /// @param separator 分隔符
    /// @param endstr 结束字符串
    /// @param ... 可变参数
    /// @return 格式化后的字符串
    std::string fc_with_ctrl(const std::string &separator, const std::string &endstr, ...);

    /// @brief 使用 C 风格的格式化功能，不对参数进行处理
    /// @param first 第一参数
    /// @param ... 可变参数
    /// @return 格式化后的字符串
    std::string fc_direct(const char *first, ...);

    /// @brief 使用 C 风格的格式化功能
    /// @param format 格式化字符串
    /// @param ... 可变参数
    /// @return 格式化后的字符串
    std::string fc(const char *format, ...);
}

#ifdef old_str_function
#define str(...) old_str_function(__VA_ARGS__)
#endif // old_str_function

#endif // __F_STRING_H__
