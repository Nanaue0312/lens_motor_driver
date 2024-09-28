#include "f_string.h"
#include <string.h>

namespace utools::fmt
{
    std::string f() { return ""; }

    std::string f_with_ctrl(const std::string &separator, const std::string &endstr)
    {
        return "";
    }

    std::string fc_with_ctrl(const std::string &separator, const std::string &endstr, ...)
    {
        std::string result;

        // 处理可变参数
        va_list args;
        va_start(args, endstr);

        const char *arg = nullptr;

        // 添加第一个参数
        if ((arg = va_arg(args, const char *)) != nullptr)
        {
            result.append(arg); // 添加参数
        }

        // 添加剩余参数
        while ((arg = va_arg(args, const char *)) != nullptr)
        {
            result.append(separator).append(arg); // 添加分隔符和下一个参数
        }

        va_end(args);

        // 添加结束字符串
        if (!endstr.empty())
        {
            result.append(endstr);
        }

        return result; // 返回格式化后的字符串
    }

    std::string fc_direct(const char *first, ...)
    {
        std::string result;

        // 处理可变参数
        va_list args;
        va_start(args, first);

        const char *arg = first;

        // 如果第一个参数为 nullptr，返回空字符串
        if (arg == nullptr)
        {
            return "";
        }

        // 添加第一个参数
        result.append(arg);

        // 添加剩余参数
        while ((arg = va_arg(args, const char *)) != nullptr)
        {
            result.append(arg); // 添加下一个参数
        }

        va_end(args);

        return result; // 返回格式化后的字符串
    }

    std::string fc(const char *format, ...)
    {
        va_list args;
        va_start(args, format);

        // 计算所需缓冲区大小
        int size = vsnprintf(nullptr, 0, format, args);
        va_end(args);

        if (size <= 0)
        {
            return "";
        }

        // 分配足够大的缓冲区
        std::string result(size + 1, '\0');

        va_start(args, format);
        // 格式化字符串
        vsnprintf(&result[0], result.size(), format, args);
        va_end(args);

        // 去掉末尾的空字符
        result.pop_back();

        return result;
    }
}