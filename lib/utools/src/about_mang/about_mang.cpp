#include "about_mang.h"
#include <string>
#include <cstdio>
#include <map>

namespace utools::about
{
#if UTOOLS_ABOUT_ENABLE == 1

    /// @brief 产品相关信息
    std::string name{UTOOLS_VERSION_MANG_NAME};         // 产品名称
    std::string cpu{UTOOLS_VERSION_MANG_CPU};           // CPU型号
    std::string arch{UTOOLS_VERSION_MANG_ARCH};         // 架构
    std::string arm{UTOOLS_VERSION_MANG_ARM};           // ARM架构
    std::string os{UTOOLS_VERSION_MANG_OS};             // 操作系统
    std::string compiler{UTOOLS_VERSION_MANG_COMPILER}; // 编译器

    /// @brief 版本号相关
    uint32_t major = UTOOLS_VERSION_MANG_MAJOR;                           // 主版本号
    uint32_t minor = UTOOLS_VERSION_MANG_MINOR;                           // 次版本号
    uint32_t patch = UTOOLS_VERSION_MANG_PATCH;                           // 修订号
    const std::string date{__DATE__};                                     // 编译日期
    const std::string time{__TIME__};                                     // 编译时间
    const std::string build{format_build_date_time_without_separators()}; // 构建时间

    /// @brief 软件相关相关信息
    std::string desc{UTOOLS_VERSION_MANG_DESC};           // 描述
    std::string author{UTOOLS_VERSION_MANG_AUTHOR};       // 作者
    std::string company{UTOOLS_VERSION_MANG_COMPANY};     // 公司
    std::string copyright{UTOOLS_VERSION_MANG_COPYRIGHT}; // 版权
    std::string url{UTOOLS_VERSION_MANG_URL};             // 网址
    std::string email{UTOOLS_VERSION_MANG_AUTHOR_EMAIL};  // 邮箱
    std::string license{UTOOLS_VERSION_MANG_LICENSE};     // 许可证

    /// @brief 设置识别信息
    std::string hardware_id{UTOOLS_VERSION_MANG_HARDWARE_ID}; // 设备（硬件）唯一标识
    std::string software_id{UTOOLS_VERSION_MANG_SOFTWARE_ID}; // 软件唯一标识
    std::string identify{UTOOLS_VERSION_MANG_IDENTIFY};       // 全局识别信息

    /// @brief 用户保存的其它版本信息
    utools::mem::AnyMap info;

    const std::string format_build_date_time_without_separators()
    {
        char dateTimeBuffer[20]; // 用于存储最终的日期时间字符串
        int hour, minute, second;

        std::map<std::string, std::string> monthMap = {
            {"Jan", "01"}, {"Feb", "02"}, {"Mar", "03"}, {"Apr", "04"}, {"May", "05"}, {"Jun", "06"}, {"Jul", "07"}, {"Aug", "08"}, {"Sep", "09"}, {"Oct", "10"}, {"Nov", "11"}, {"Dec", "12"}};

        char monthStr[4];
        int day, year;

        sscanf(date.c_str(), "%s %d %d", monthStr, &day, &year);

        sscanf(time.c_str(), "%d:%d:%d", &hour, &minute, &second);

        snprintf(dateTimeBuffer, sizeof(dateTimeBuffer), "%04d%02d%02d%02d%02d%02d",
                 year, std::stoi(monthMap[monthStr]), day, hour, minute, second);

        return std::string(dateTimeBuffer);
    }

    const uint32_t int_ver()
    {
        return ((major << 16) | (minor << 8) | (patch));
    }

    const std::string str_ver()
    {
        return std::to_string(major) + "." + std::to_string(minor) + "." + std::to_string(patch);
    }

    const std::string full_ver()
    {
        return str_ver() + " build " + build;
    }

#endif
}