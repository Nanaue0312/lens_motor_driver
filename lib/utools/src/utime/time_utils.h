#ifndef __TIME_UTILS_H__
#define __TIME_UTILS_H__

#include <chrono>

#include "../utools_cfg.h"

#if (UTOOLS_OS_TYPE == UTOOLS_OS_TYPE_WINDOWS || \
     UTOOLS_OS_TYPE == UTOOLS_OS_TYPE_LINUX ||   \
     UTOOLS_OS_TYPE == UTOOLS_OS_TYPE_MACOS ||   \
     UTOOLS_OS_TYPE == UTOOLS_OS_TYPE_CPP_STD)

#if UTOOLS_THREAD_FUNCATION == UTOOLS_PTHREAD_FUNCATION
#include <pthread.h>
#elif UTOOLS_THREAD_FUNCATION == UTOOLS_STD_THREAD_FUNCATION
#include <thread>
#endif

#elif UTOOLS_OS_TYPE == UTOOLS_OS_TYPE_FREERTOS
extern "C"{
    #include "FreeRTOS.h"
    #include "task.h"
}



#elif UTOOLS_OS_TYPE == UTOOLS_OS_TYPE_NULL

#if UTOOLS_EMBEDDED_PLATFORM == UTOOLS_EMBEDDED_PLATFORM_STM32
#if defined(STM32F0xx)
#include "stm32f0xx_hal.h"
#elif defined(STM32F1xx)
#include "stm32f1xx_hal.h"
#elif defined(STM32F2xx)
#include "stm32f2xx_hal.h"
#elif defined(STM32F3xx)
#include "stm32f3xx_hal.h"
#elif defined(STM32F4xx)
#include "stm32f4xx_hal.h"
#elif defined(STM32F7xx)
#include "stm32f7xx_hal.h"
#elif defined(STM32G0xx)
#include "stm32g0xx_hal.h"
#elif defined(STM32G4xx)
#include "stm32g4xx_hal.h"
#elif defined(STM32H7xx)
#include "stm32h7xx_hal.h"
#elif defined(STM32L0xx)
#include "stm32l0xx_hal.h"
#elif defined(STM32L1xx)
#include "stm32l1xx_hal.h"
#elif defined(STM32L4xx)
#include "stm32l4xx_hal.h"
#elif defined(STM32L5xx)
#include "stm32l5xx_hal.h"
#elif defined(STM32U5xx)
#include "stm32u5xx_hal.h"
#elif defined(STM32WBxx)
#include "stm32wbxx_hal.h"
#elif defined(STM32WLxx)
#include "stm32wlxx_hal.h"

#elif UTOOLS_EMBEDDED_PLATFORM == UTOOLS_EMBEDDED_PLATFORM_ESP32
#include "FreeRTOS.h"
#include "task.h"
#include "esp_timer.h"

#else
#warning "Unsupported platform!"

#endif // UTOOLS_EMBEDDED_PLATFORM
#endif // UTOOLS_OS_TYPE == UTOOLS_OS_TYPE_NULL
#endif // UTOOLS_OS_TYPE

namespace utools::time
{
    /// @brief 获取unix时间戳
    /// @tparam _Period 指定需要的类型
    /// @return 时间戳（在mcu等平台上，此时间也boot_timsetamp可能是相同的）
    template <typename _Period = std::chrono::milliseconds>
    const int64_t unix_ts()
    {
#if (UTOOLS_OS_TYPE == UTOOLS_OS_TYPE_WINDOWS || \
     UTOOLS_OS_TYPE == UTOOLS_OS_TYPE_LINUX ||   \
     UTOOLS_OS_TYPE == UTOOLS_OS_TYPE_MACOS ||   \
     UTOOLS_OS_TYPE == UTOOLS_OS_TYPE_CPP_STD)
        return std::chrono::duration_cast<_Period>(std::chrono::system_clock::now().time_since_epoch()).count();

#elif UTOOLS_OS_TYPE == UTOOLS_OS_TYPE_FREERTOS
        uint32_t milliseconds = xTaskGetTickCount();

#elif UTOOLS_OS_TYPE == UTOOLS_OS_TYPE_NULL
#if UTOOLS_EMBEDDED_PLATFORM == UTOOLS_EMBEDDED_PLATFORM_STM32
        uint32_t milliseconds = HAL_GetTick();
#elif UTOOLS_EMBEDDED_PLATFORM == UTOOLS_EMBEDDED_PLATFORM_ESP32
        uint32_t milliseconds = esp_timer_get_time() / 1000;
#endif

#else
        uint32_t milliseconds = 0;
#endif // UTOOLS_OS_TYPE
        if constexpr (std::is_same<_Period, std::chrono::nanoseconds>::value)
        {
            return milliseconds * 1000000;
        }
        if constexpr (std::is_same<_Period, std::chrono::microseconds>::value)
        {
            return milliseconds * 1000;
        }
        else if constexpr (std::is_same<_Period, std::chrono::milliseconds>::value)
        {
            return milliseconds;
        }
        else if constexpr (std::is_same<_Period, std::chrono::seconds>::value)
        {
            return milliseconds / 1000;
        }
        else if constexpr (std::is_same<_Period, std::chrono::minutes>::value)
        {
            return milliseconds / (1000 * 60);
        }
        else if constexpr (std::is_same<_Period, std::chrono::hours>::value)
        {
            return milliseconds / (1000 * 60 * 60);
        }
        else
        {
            return milliseconds;
        }
    }

    /// @brief 获取开机时间戳
    /// @tparam _Period 指定需要的类型
    /// @return 开始时间长度
    template <typename _Period = std::chrono::milliseconds>
    const int64_t boot_ts()
    {
#if (UTOOLS_OS_TYPE == UTOOLS_OS_TYPE_WINDOWS || \
     UTOOLS_OS_TYPE == UTOOLS_OS_TYPE_LINUX ||   \
     UTOOLS_OS_TYPE == UTOOLS_OS_TYPE_MACOS ||   \
     UTOOLS_OS_TYPE == UTOOLS_OS_TYPE_CPP_STD)
        return std::chrono::duration_cast<_Period>(std::chrono::steady_clock::now().time_since_epoch()).count();
#else
        return unix_ts<_Period>();
#endif
    }

    void nsleep(uint32_t nseconds);
    void usleep(uint32_t useconds);
    void msleep(uint32_t mseconds);
    void sleep(uint32_t seconds);

    /// @brief 使用循环的方式实现延时，不推荐使用
    /// @param nseconds 纳秒
    /// @warning 不推荐使用，此定时器精度差
    inline void soft_nsleep(const int64_t &nseconds)
    {
#if UTOOLS_NANOSECONDS_SLEEP_ENABLE == 1
        if (nseconds > 0)
        {
            volatile auto start_ts = boot_ts<std::chrono::nanoseconds>();
            while (boot_ts<std::chrono::nanoseconds>() - start_ts < nseconds)
            {
                ;
            }
        }
#endif // UTOOLS_NANOSECONDS_SLEEP_ENABLE
    }

    /// @brief 延时指定的时长
    /// @tparam _Rep 时间单位
    /// @tparam _Period 时间单位
    /// @param __rtime 延时时间
    template <typename _Rep, typename _Period>
    inline void sleep_for(const std::chrono::duration<_Rep, _Period> &__rtime)
    {
        if (__rtime <= __rtime.zero())
            return;

#if UTOOLS_THREAD_FUNCATION == UTOOLS_STD_THREAD_FUNCATION
        std::this_thread::sleep_for(__rtime);

#elif UTOOLS_THREAD_FUNCATION == UTOOLS_FREERTOS_TASK_FUNCATION
        auto __ms = std::chrono::duration_cast<std::chrono::milliseconds>(__rtime);
        // 处理毫秒延时
        vTaskDelay(__ms.count() / portTICK_PERIOD_MS); // 毫秒延时
        // 处理纳秒延时
        auto __ns = std::chrono::duration_cast<std::chrono::nanoseconds>(__rtime - __ms);
        soft_nsleep(__ns.count());

#elif UTOOLS_THREAD_FUNCATION == UTOOLS_PTHREAD_FUNCATION
        struct timespec ts;
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(__rtime).count();
        ts.tv_sec = ms / 1000;
        ts.tv_nsec = (ms % 1000) * 1000000;
        nanosleep(&ts, nullptr);

#elif UTOOLS_THREAD_FUNCATION == UTOOLS_THREAD_FUNCATION_NUL
/// @note 如果不支持线程，尝试使用与硬件相关的延时函数
#if UTOOLS_EMBEDDED_PLATFORM == UTOOLS_EMBEDDED_PLATFORM_STM32
        auto __ms = std::chrono::duration_cast<std::chrono::milliseconds>(__rtime);
        HAL_Delay(__ms.count());
        auto __ns = std::chrono::duration_cast<std::chrono::nanoseconds>(__rtime - __ms);
        soft_nsleep(__ns.count());

#elif UTOOLS_EMBEDDED_PLATFORM == UTOOLS_EMBEDDED_PLATFORM_ESP32
        // 延时微秒的esp32实现
        auto __us = std::chrono::duration_cast<std::chrono::microseconds>(__rtime).count();
        volatile uint32_t count = 0;
        uint32_t target = esp_timer_get_time() + __us;
        while (esp_timer_get_time() < target)
        {
            count++; // 空循环以消耗时间
        }
        // 延时纳秒的esp32实现
        auto __ns = std::chrono::duration_cast<std::chrono::nanoseconds>(__rtime - __us);
        soft_nsleep(__ns.count());

#endif // UTOOLS_OS_TYPE == UTOOLS_OS_TYPE_NULL
#endif
    }

    /// @brief 睡眠到指定的时间
    /// @tparam _Clock 时钟类型
    /// @tparam _Duration 参数时间类型
    /// @param __atime 指定的时间点
    template <typename _Clock, typename _Duration>
    inline void sleep_until(const std::chrono::time_point<_Clock, _Duration> &__atime)
    {
        auto __now = _Clock::now();
        if (_Clock::is_steady)
        {
            if (__now < __atime)
                sleep_for(__atime - __now);
            return;
        }
        while (__now < __atime)
        {
            sleep_for(__atime - __now);
            __now = _Clock::now();
        }
    }

    /// @brief 睡眠指定的时间
    /// @param nseconds 纳秒数
    /// @warning 如果需要使用高精度延时，请使用其它方法实现，如硬件定义器
    inline void nsleep(uint32_t nseconds)
    {
        sleep_for(std::chrono::nanoseconds(nseconds));
    }

    /// @brief 睡眠指定的时间
    /// @param useconds 微秒数，microseconds
    /// @warning 如果需要使用高精度延时，请使用其它方法实现，如硬件定义器
    inline void usleep(uint32_t useconds)
    {
        sleep_for(std::chrono::microseconds(useconds));
    }

    /// @brief 睡眠指定的时间
    /// @param mseconds 毫秒数
    inline void msleep(uint32_t mseconds)
    {
        sleep_for(std::chrono::milliseconds(mseconds));
    }

    /// @brief 睡眠指定的时间
    /// @param seconds 秒数
    inline void sleep(uint32_t seconds)
    {
        sleep_for(std::chrono::seconds(seconds));
    }
}

#endif // __TIME_UTILS_H__
