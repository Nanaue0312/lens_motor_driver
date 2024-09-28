/// @brief 时间功能配置器
///        为了适应不同的系统，需要对时间函数进行重新配置，本文件提供了默认配置

#include "../utools_cfg.h"

/// @brief 如果使用了FreeRTOS，可以通过UTOOLS_FREERTOS_STD_SLEEP_FUNC_ENABLE宏开启
/// 标准库中对std::this_thread::sleep_for/sleep_until支持
#if UTOOLS_FREERTOS_STD_SLEEP_FUNC_ENABLE == 1

#include "FreeRTOS.h"
#include "task.h"

extern "C" void usleep(unsigned int useconds)
{
    const TickType_t ticks = useconds / (1000000 / configTICK_RATE_HZ);
    if (ticks > 0)
    {
        vTaskDelay(ticks);
    }
}

extern "C" unsigned int sleep(unsigned int seconds)
{
    const TickType_t ticks = seconds * configTICK_RATE_HZ;
    vTaskDelay(ticks);
    return 0;
}

#endif // UTOOLS_FREERTOS_STD_SLEEP_FUNC_ENABLE

/// @brief 基于freertos系统，实现_gettimeofday函数
#if UTOOLS_FREERTOS__GETTOPICTIME_ENABLE == 1
#include "FreeRTOS.h"
#include "task.h"
#include <sys/time.h>

extern "C" int _gettimeofday(struct timeval *tv, void *tzvp)
{
    // 获取当前的Tick计数
    TickType_t ticks = xTaskGetTickCount();
    // 将Tick计数转换为微秒
    uint64_t time_us = (uint64_t)ticks * 1000000 / configTICK_RATE_HZ;

    if (tv)
    {
        tv->tv_sec = time_us / 1000000;  // 转换为秒
        tv->tv_usec = time_us % 1000000; // 剩余的微秒
    }
    return 0;
}
#endif // UTOOLS_FREERTOS__GETTOPICTIME_ENABLE
