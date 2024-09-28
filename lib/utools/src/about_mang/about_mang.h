#ifndef __UTOOLS_ABOUT_MANG_H__
#define __UTOOLS_ABOUT_MANG_H__

#include <stdint.h>
#include <map>

#include "../any_store/any_type_map.h"
#include "about_mang_def.h"

namespace utools::about
{
#if UTOOLS_ABOUT_ENABLE == 1

    /// @brief 获取版本号
    /// @return 输出“20230115153045”类型的字符串
    const std::string format_build_date_time_without_separators();

    /// @brief 获取版本号
    /// @return uint32_t类型的版本号，可用于比较大小等操作
    const uint32_t int_ver();

    /// @brief 获取版本号
    /// @return std::string类型的版本号，如“1.2.3”
    const std::string str_ver();

    /// @brief 获取版本号
    /// @return std::string类型的版本号，如“1.2.3 build 20230115153045”
    const std::string full_ver();

    /// @brief 产品相关信息
    extern std::string name;     // 产品名称
    extern std::string cpu;      // CPU型号
    extern std::string arch;     // 架构
    extern std::string arm;      // ARM架构
    extern std::string os;       // 操作系统
    extern std::string compiler; // 编译器

    /// @brief 版本号相关
    extern uint32_t major;          // 主版本号
    extern uint32_t minor;          // 次版本号
    extern uint32_t patch;          // 修订号
    extern const std::string date;  // 编译日期
    extern const std::string time;  // 编译时间
    extern const std::string build; // 构建时间

    /// @brief 软件相关相关信息
    extern std::string desc;      // 描述
    extern std::string author;    // 作者
    extern std::string company;   // 公司
    extern std::string copyright; // 版权
    extern std::string url;       // 网址
    extern std::string email;     // 邮箱
    extern std::string license;   // 许可证

    /// @brief 设置识别信息
    extern std::string hardware_id; // 设备（硬件）唯一标识
    extern std::string software_id; // 软件唯一标识
    extern std::string identify;    // 全局识别信息

    /// @brief 用户保存的其它版本信息
    extern utools::mem::AnyMap info;

    /// @brief 设置一个指定的值
    /// @tparam T 值类型
    /// @tparam K 键类型
    /// @param key 键
    template <typename T, typename K>
    utools::mem::AnyMap &set(const K &key, const T &value)
    {
        return info.set(key, value);
    }

    /// @brief 获取一个指定的值
    /// @tparam T 值类型
    /// @tparam K 键类型
    /// @param key 健值
    /// @return 指定key对应的值，如果没有找到会报错
    template <typename T, typename K>
    T get(const K &key)
    {
        return info.get<T>(key);
    }

    /// @brief 获取指定key对应的值，如果key不存在，则返回默认值
    /// @tparam T 值类型
    /// @tparam K 键类型
    /// @param key 键
    /// @param defaule_val 默认值
    /// @return 指定key对应的值，如果key不存在，则返回默认值
    template <typename T, typename K>
    T get(const K &key, const T &defaule_val)
    {
        return info.get<T>(key, defaule_val);
    }

#endif
}

#endif // __UTOOLS_VERSION_MANG_H__