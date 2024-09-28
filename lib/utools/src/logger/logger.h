#ifndef __LOGGER_H__
#define __LOGGER_H__

#include <functional>
#include <string>
#include <array>
#include <type_traits>

#if UTOOLS_LOGGER_FILTER_ENABLE == 1 // 开启过滤功能后可用
#include <vector>
#include <mutex>
#endif

#include "../utime/time_utils.h"
#include "../ufmt/f_string.h"
#include "../ufmt/str_conv.h"
#include "../utools_macro.h"
#include "../utools_cfg.h"

namespace utools
{
#define UTOOLS_LOGGER_INFO_STR "INFO "
#define UTOOLS_LOGGER_TRACE_STR "TRACE"
#define UTOOLS_LOGGER_DEBUG_STR "DEBUG"
#define UTOOLS_LOGGER_WARN_STR "WARN "
#define UTOOLS_LOGGER_ERROR_STR "ERROR"
#define UTOOLS_LOGGER_FATAL_STR "FATAL"

#define MAKE_LOCATION_STRING(file, line) "[" file ":" TOSTRING(line) "] "
#define LOCATION_STRING MAKE_LOCATION_STRING(__FILE__, __LINE__)

// 如果系统没有配置时间戳函数，则不打印时间戳
#if UTOOLS_LOGGER_TIMESTAMP_ENABLE == 1
#define UTOOLS_LOGGER_HEADER(logger_level_str) ("[" + utools::logger::format_timestamp() + "] [" logger_level_str "] ")
#else
#define UTOOLS_LOGGER_HEADER(logger_level_str) ("[" logger_level_str "] ")
#endif

    /// @brief logger实现
    class logger
    {
    public:
        /// @brief 日志等级
        enum class level
        {
            NONE = 0,  // 不打印
            INFO = 1,  // 信息
            TRACE = 2, // 追踪
            DEBUG = 3, // 调试
            WARN = 4,  // 警告
            ERROR = 5, // 错误
            FATAL = 6, // 致命错误
        };

        /// @brief 打印日志，此功能不会判断打印等级，会直接输出
        /// @tparam ...Args 参数类型
        /// @param ...args 参数值
        template <typename... Args>
        static void log(const Args &...args)
        {
            __print(args...);
        }

        template <typename... Args>
        static void print(const Args &...args)
        {
            __print(args...);
        }

        template <typename... Args>
        static void info(const Args &...args)
        {
            if (__log_levels[static_cast<size_t>(level::INFO)])
            {
                __print(UTOOLS_LOGGER_HEADER(UTOOLS_LOGGER_INFO_STR),
                        args...);
            }
        }

        template <typename... Args>
        static void trace(const Args &...args)
        {
            if (__log_levels[static_cast<size_t>(level::TRACE)])
            {
                __print(UTOOLS_LOGGER_HEADER(UTOOLS_LOGGER_TRACE_STR),
                        args...);
            }
        }

        template <typename... Args>
        static void debug(const Args &...args)
        {
            if (__log_levels[static_cast<size_t>(level::DEBUG)])
            {
                __print(UTOOLS_LOGGER_HEADER(UTOOLS_LOGGER_DEBUG_STR),
                        args...);
            }
        }

        template <typename... Args>
        static void warn(const Args &...args)
        {
            if (__log_levels[static_cast<size_t>(level::WARN)])
            {
                __print(UTOOLS_LOGGER_HEADER(UTOOLS_LOGGER_WARN_STR),
                        args...);
            }
        }

        template <typename... Args>
        static void error(const Args &...args)
        {
            if (__log_levels[static_cast<size_t>(level::ERROR)])
            {
                __print(UTOOLS_LOGGER_HEADER(UTOOLS_LOGGER_ERROR_STR),
                        args...);
            }
        }

        template <typename... Args>
        static void fatal(const Args &...args)
        {
            if (__log_levels[static_cast<size_t>(level::FATAL)])
            {
                __print(UTOOLS_LOGGER_HEADER(UTOOLS_LOGGER_FATAL_STR),
                        args...);
            }
        }

        /// @brief 同时设置多个不同的等级
        /// @param levels 需要设置的等级
        static void set_log_levels(std::initializer_list<level> levels)
        {
            for (auto level : levels)
            {
                set_log_level(level);
            }
        }

        /// @brief 获取level配置
        /// @return 所有的level配置
        static const std::array<bool, 7> &get_log_levels()
        {
            return __log_levels;
        }

        /// @brief 判断指定的日志等级是否已经设置
        /// @param level 需要判断的等级
        /// @return true表示已经设置，false表示未设置
        static bool level_is_set(level level)
        {
            return __log_levels[static_cast<size_t>(level)];
        }

        /// @brief 设置日志打印等级
        /// @param level 需要设置的等级
        static void set_log_level(level level)
        {
            __log_levels[static_cast<size_t>(level)] = true;
        }

        /// @brief 设置所有level的打印状态
        /// @param enable true表示开启，false表示关闭
        static void set_log_level_all(bool enable = true)
        {
            __log_levels.fill(enable);
        }

        /// @brief 清除指定的日志等级
        /// @param level 需要清除的等级
        static void clear_log_level(level level)
        {
            __log_levels[static_cast<size_t>(level)] = false;
        }

        /// @brief 清除指定的日志等级
        /// @param levels 需要清除的等级
        static void clear_log_levels(std::initializer_list<level> levels)
        {
            for (auto &level : levels)
            {
                clear_log_level(level);
            }
        }

        /// @brief 设置打印相关的函数
        /// @param print_fun 打印函数
        static void set_log_fun(const std::function<void(const char *)> &print_fun)
        {
            __print_fun = print_fun;
        }

        /// @brief 设置打印相关的函数
        /// @param func 类成员函数
        /// @param obj 类对象指针
        /// @param args 可变参数
        /// @return 无
        template <typename ReturnType, typename T>
        static void bind_print(ReturnType (T::*func)(const char *), T *obj)
        {
            __print_fun = std::bind(func, obj, std::placeholders::_1);
        }

        /// @brief 设置打印相关的函数
        /// @param func 静态函数或自由函数
        /// @param args 可变参数
        /// @return 无
        template <typename ReturnType>
        static void bind_print(ReturnType (*func)(const char *))
        {
            __print_fun = std::bind(func, std::placeholders::_1);
        }

        /// @brief 设置自动换行模式
        /// @param auto_line_break 自动换行模式的开关。true表示开启，false表示关闭。
        static void set_auto_line_break(bool auto_line_break)
        {
            if (auto_line_break)
            {
                __end_str = "\n";
            }
            else
            {
                __end_str = "";
            }
        }

        /// @brief 设置用户连续输入时，是否在每一个参数后增加字符串
        /// @param spacer_str 打印参数后增加的字符串，默认为空格
        static void set_spacer(const std::string &spacer_str = " ")
        {
            __space_str = spacer_str;
        }

        /// @brief 获取当前时间戳
        /// @return 当前时间戳
        static std::string format_timestamp()
        {
            auto ts = ::utools::time::unix_ts();
            long long seconds = ts / 1000;
            uint32_t milliseconds = ts % 1000;
            char xiaoshu[]{'.', '0', '0', '0', 0};
            for (int i = 3; i >= 1; --i)
            {
                xiaoshu[i] = '0' + (milliseconds % 10);
                milliseconds /= 10;
            }
            return std::to_string(seconds) + std::string(xiaoshu);
        }

        static void add_filter(const std::string &filter_str)
        {
#if UTOOLS_LOGGER_FILTER_ENABLE == 1 // 开启过滤功能后可用
            if (filter_str.empty())
            {
                return;
            }
            {
                std::lock_guard<std::mutex> guard(__filter_mutex);
                if (std::find(__filter_keywords.begin(), __filter_keywords.end(), filter_str) == __filter_keywords.end())
                {
                    __filter_keywords.push_back(filter_str);
                }
            }
#endif
        }

        static void add_filters(std::initializer_list<std::string> filter_strs)
        {
#if UTOOLS_LOGGER_FILTER_ENABLE == 1 // 开启过滤功能后可用
            for (const auto &filter_str : filter_strs)
            {
                add_filter(filter_str);
            }
#endif
        }

        static void remove_filter(const std::string &filter_str)
        {
#if UTOOLS_LOGGER_FILTER_ENABLE == 1 // 开启过滤功能后可用

            std::lock_guard<std::mutex> guard(__filter_mutex);
            __filter_keywords.erase(std::remove(__filter_keywords.begin(), __filter_keywords.end(), filter_str), __filter_keywords.end());
#endif
        }

        static void clear_filters()
        {
#if UTOOLS_LOGGER_FILTER_ENABLE == 1 // 开启过滤功能后可用
            std::lock_guard<std::mutex> guard(__filter_mutex);
            __filter_keywords.clear();
#endif
        }

    private:
        logger() = default;
        virtual ~logger() {}

        template <typename... Args>
        static void __print(const std::string &header, const Args &...args)
        {
#if UTOOLS_LOGGER_ENABLE == 1
            std::string log_str = header;
            log_str += ::utools::fmt::fc_with_ctrl(__space_str, __end_str, (utools::fmt::to_string(args).c_str())..., nullptr);

            // log_str += ::utools::fmt::f_with_ctrl(__space_str, __end_str, args...); // 会增加内容占用
#if UTOOLS_LOGGER_FILTER_ENABLE == 1
            {
                std::lock_guard<std::mutex> guard(__filter_mutex);
                if (!__filter_keywords.empty())
                {
                    for (const auto &fliter_keyword : __filter_keywords)
                    {
                        if (log_str.find(fliter_keyword) != std::string::npos)
                        {
                            return; // 如果找到需要过滤的关键字，则不打印
                        }
                    }
                }
            }
#endif // UTOOLS_LOGGER_FILTER_ENABLE
            __print_fun(log_str.c_str());
#endif // UTOOLS_LOGGER_ENABLE
        }

        static std::function<void(const char *)> __print_fun;
        static std::array<bool, 7> __log_levels;
        static std::string __end_str;
        static std::string __space_str;

#if UTOOLS_LOGGER_FILTER_ENABLE == 1                       // 开启过滤功能后可用
        static std::vector<std::string> __filter_keywords; // 过滤关键字
        static std::mutex __filter_mutex;                  // 用于保护 __filter_keywords
#endif
    };

/// @brief 打印有文件位置和行号的日志
#define UTPRINT(...) ::utools::logger::log(__VA_ARGS__)
#define UTINFO(...) ::utools::logger::log(UTOOLS_LOGGER_HEADER(UTOOLS_LOGGER_INFO_STR) + LOCATION_STRING, ##__VA_ARGS__)
#define UTTRACE(...) ::utools::logger::log(UTOOLS_LOGGER_HEADER(UTOOLS_LOGGER_TRACE_STR) + LOCATION_STRING, ##__VA_ARGS__)
#define UTDEBUG(...) ::utools::logger::log(UTOOLS_LOGGER_HEADER(UTOOLS_LOGGER_DEBUG_STR) + LOCATION_STRING, ##__VA_ARGS__)
#define UTWARN(...) ::utools::logger::log(UTOOLS_LOGGER_HEADER(UTOOLS_LOGGER_WARN_STR) + LOCATION_STRING, ##__VA_ARGS__)
#define UTERROR(...) ::utools::logger::log(UTOOLS_LOGGER_HEADER(UTOOLS_LOGGER_ERROR_STR) + LOCATION_STRING, ##__VA_ARGS__)
#define UTFATAL(...) ::utools::logger::log(UTOOLS_LOGGER_HEADER(UTOOLS_LOGGER_FATAL_STR) + LOCATION_STRING, ##__VA_ARGS__)

/// @brief 打印有文件位置和行号的日志
#define logger_print(...) logger::log(__VA_ARGS__)
#define logger_info(...) logger::log(UTOOLS_LOGGER_HEADER(UTOOLS_LOGGER_INFO_STR) + LOCATION_STRING, ##__VA_ARGS__)
#define logger_trace(...) logger::log(UTOOLS_LOGGER_HEADER(UTOOLS_LOGGER_TRACE_STR) + LOCATION_STRING, ##__VA_ARGS__)
#define logger_debug(...) logger::log(UTOOLS_LOGGER_HEADER(UTOOLS_LOGGER_DEBUG_STR) + LOCATION_STRING, ##__VA_ARGS__)
#define logger_warn(...) logger::log(UTOOLS_LOGGER_HEADER(UTOOLS_LOGGER_WARN_STR) + LOCATION_STRING, ##__VA_ARGS__)
#define logger_error(...) logger::log(UTOOLS_LOGGER_HEADER(UTOOLS_LOGGER_ERROR_STR) + LOCATION_STRING, ##__VA_ARGS__)
#define logger_fatal(...) logger::log(UTOOLS_LOGGER_HEADER(UTOOLS_LOGGER_FATAL_STR) + LOCATION_STRING, ##__VA_ARGS__)

} // namespace utools

#endif // __LOGGER_H__
