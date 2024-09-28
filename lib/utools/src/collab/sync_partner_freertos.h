/// @brief 同步伙伴类，用于FreeRTOS
///        使用互斥锁和条件变量实现同步
///        IS_ISR为true时，wait函数在ISR中调用
///        IS_ISR为false时，wait函数在任务中调用
///        wait函数会阻塞，直到条件变量被释放
///        notify函数唤醒等待的线程

#ifndef __SYNC_PARTNER_FREERTOS_H__
#define __SYNC_PARTNER_FREERTOS_H__

#if UTOOLS_COLLAB_SYNC_PARTNER_ENABLE == 1

extern "C"
{
#include "FreeRTOS.h"
#include "semphr.h"
}

namespace utools::collab::freertos
{
    template <bool WAIT_IS_ISR = false, bool NOTIFY_IS_ISR = false>
    class SyncPartner
    {
    public:
        SyncPartner()
        {
            // 创建互斥锁
            __mutex = xSemaphoreCreateMutex();
            configASSERT(__mutex);
            // 创建二元信号量（条件变量）
            __condition = xSemaphoreCreateBinary();
            configASSERT(__condition);
            // 初始化二元信号量为已释放状态
            xSemaphoreGive(__condition);
        }

        virtual ~SyncPartner()
        {
            vSemaphoreDelete(__mutex);
            vSemaphoreDelete(__condition);
        }

        /// @brief 等待条件变量
        void wait()
        {
            if constexpr (WAIT_IS_ISR)
            {
                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                xSemaphoreTakeFromISR(__condition, &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
            else
            {
                xSemaphoreTake(__mutex, portMAX_DELAY); // 获取互斥锁
                xSemaphoreTake(__condition, portMAX_DELAY);
                xSemaphoreGive(__mutex); // 释放互斥锁
            }
        }

        /// @brief 通知等待的线程条件已满足
        void notify()
        {
            if constexpr (NOTIFY_IS_ISR)
            {
                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                xSemaphoreGiveFromISR(__condition, &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
            else
            {
                xSemaphoreTake(__mutex, portMAX_DELAY); // 获取互斥锁
                xSemaphoreGive(__condition);            // 释放条件变量
                xSemaphoreGive(__mutex);                // 释放互斥锁
            }
        }

        /// @brief 手动释放信号量
        void release()
        {
            if constexpr (WAIT_IS_ISR || NOTIFY_IS_ISR)
            {
                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                xSemaphoreGiveFromISR(__condition, &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
            else
            {
                xSemaphoreGive(__condition);
            }
        }

    private:
        SemaphoreHandle_t __mutex;
        SemaphoreHandle_t __condition;
    };

    template <bool WAIT_IS_ISR = false, bool NOTIFY_IS_ISR = false>
    SyncPartner<WAIT_IS_ISR, NOTIFY_IS_ISR> make_sync_partner()
    {
        return SyncPartner<WAIT_IS_ISR, NOTIFY_IS_ISR>();
    }
}
#endif // UTOOLS_COLLAB_SYNC_PARTNER_ENABLE
#endif // __SYNC_PARTNER_FREERTOS_H__