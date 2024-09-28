#ifndef __UTOOLS_CFG_DEF_H__
#define __UTOOLS_CFG_DEF_H__

// 功能配置开关，1:启用 0:禁用
#define UTOOLS_ENABLE 1
#define UTOOLS_DISABLE 0

// 操作系统类型
#define UTOOLS_OS_TYPE_NULL 0     // 无操作系统
#define UTOOLS_OS_TYPE_CPP_STD 1  // 使用C++标准库的系统
#define UTOOLS_OS_TYPE_WINDOWS 2  // Windows
#define UTOOLS_OS_TYPE_LINUX 3    // Linux
#define UTOOLS_OS_TYPE_MACOS 4    // MacOS
#define UTOOLS_OS_TYPE_FREERTOS 5 // FreeRTOS
#define UTOOLS_OS_TYPE_RTTHREAD 6 // RT-Thread
#define UTOOLS_OS_TYPE_UCOS 7     // UCOS
#define UTOOLS_OS_TYPE_CMSIS_V2 8 // CMSIS V2，是一种中间层系统，常见于STM32

// 嵌入式系统平台
#define UTOOLS_EMBEDDED_PLATFORM_NULL 0  // 非嵌入式平台
#define UTOOLS_EMBEDDED_PLATFORM_STM32 1 // STM32
#define UTOOLS_EMBEDDED_PLATFORM_ESP32 2 // ESP32

// 定义系统支持的thread功能 1: std::thread, 2: freertos task, 3: pthread
#define UTOOLS_THREAD_FUNCATION_NULL 0   // 没有可使用的线程管理函数
#define UTOOLS_STD_THREAD_FUNCATION 1    // 使用std::thread
#define UTOOLS_FREERTOS_TASK_FUNCATION 2 // 使用freertos task
#define UTOOLS_PTHREAD_FUNCATION 3       // 使用pthread
#define UTOOLS_OSTHREADNEW_FUNCATION 4   // 使用osThreadNew

#endif // __UTOOLS_CFG_DEF_H__