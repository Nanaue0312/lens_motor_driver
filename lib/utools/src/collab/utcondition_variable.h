#ifndef __UTOOLS_UTCONDITION_VARIABLE_H__
#define __UTOOLS_UTCONDITION_VARIABLE_H__

#include "../utools_cfg.h"
#include "utmutex.h"
#include <chrono>

#if UTOOLS_THREAD_FUNCATION == UTOOLS_STD_THREAD_FUNCATION
#include <condition_variable>
#include <mutex>
#elif UTOOLS_THREAD_FUNCATION == UTOOLS_PTHREAD_FUNCATION
#include <pthread.h>
#elif UTOOLS_THREAD_FUNCATION == UTOOLS_FREERTOS_TASK_FUNCATION
#include "FreeRTOS.h"
#include "semphr.h"
#else
#warning "not support thread function"
#endif

namespace utools::collab
{
    class ConditionVariable
    {
    public:
        ConditionVariable()
        {
#if UTOOLS_THREAD_FUNCATION == UTOOLS_FREERTOS_TASK_FUNCATION
            semaphore = xSemaphoreCreateBinary();
            xSemaphoreGive(semaphore); // 初始化为可用状态
#elif UTOOLS_THREAD_FUNCATION == UTOOLS_PTHREAD_FUNCATION
            pthread_cond_init(&cond, nullptr);
            pthread_mutex_init(&mutex, nullptr);
#endif
        }

        ~ConditionVariable()
        {
#if UTOOLS_THREAD_FUNCATION == UTOOLS_FREERTOS_TASK_FUNCATION
            vSemaphoreDelete(semaphore);
#elif UTOOLS_THREAD_FUNCATION == UTOOLS_PTHREAD_FUNCATION
            pthread_cond_destroy(&cond);
            pthread_mutex_destroy(&mutex);
#endif
        }

        void wait(UniqueLock &lock)
        {
#if UTOOLS_THREAD_FUNCATION == UTOOLS_STD_THREAD_FUNCATION
            cond_var.wait(lock);
#elif UTOOLS_THREAD_FUNCATION == UTOOLS_PTHREAD_FUNCATION
            lock.unlock();
            pthread_cond_wait(&cond, &mutex);
#elif UTOOLS_THREAD_FUNCATION == UTOOLS_FREERTOS_TASK_FUNCATION
            lock.unlock();
            xSemaphoreTake(semaphore, portMAX_DELAY);
#endif
        }

        template <typename ReturnType, typename... Args>
        void wait(UniqueLock &lock, ReturnType (*pred)(Args...), Args... args)
        {
            if (pred(std::forward<Args>(args)...))
            {
                return;
            }
            wait(lock);
        }

        template <typename ReturnType, typename T, typename... Args>
        void wait(UniqueLock &lock, ReturnType (T::*pred)(Args...), T *obj, Args &&...args)
        {
            if (obj->pred(std::forward<Args>(args)...))
            {
                return;
            }
            wait(lock);
        }

        template <typename Func, typename... Args>
        void wait(UniqueLock &lock, Func &&pred, Args &&...args)
        {
            if (pred(std::forward<Args>(args)...))
            {
                return;
            }
            wait(lock);
        }

        /// @brief 等待条件变量，直到超时或条件满足
        /// @param mtx 互斥锁
        /// @param timeout_duration 超时时间
        /// @return true 表示条件满足，false 表示超时
        template <typename Rep, typename Period>
        bool wait_for(UniqueLock &lock, const std::chrono::duration<Rep, Period> &timeout_duration)
        {
#if UTOOLS_THREAD_FUNCATION == UTOOLS_STD_THREAD_FUNCATION
            return cond_var.wait_for(lock, timeout_duration) == std::cv_status::no_timeout; // 返回是否成功
#elif UTOOLS_THREAD_FUNCATION == UTOOLS_PTHREAD_FUNCATION
            int millseconds = std::chrono::duration_cast<std::chrono::milliseconds>(timeout_duration).count();
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_sec += milliseconds / 1000;
            ts.tv_nsec += (milliseconds % 1000) * 1000000;

            lock.unlock();
            int result = pthread_cond_timedwait(&cond, &mutex, &ts);
            return (result == 0); // 返回是否成功
#elif UTOOLS_THREAD_FUNCATION == UTOOLS_FREERTOS_TASK_FUNCATION
            TickType_t timeout = std::chrono::duration_cast<std::chrono::milliseconds>(timeout_duration).count();
            lock.unlock();
            bool result = xSemaphoreTake(semaphore, timeout); // 等待条件，带超时
            return result;
#endif
            return false; // 如果没有成功等待，则返回 false
        }

        template <typename Rep, typename Period, typename ReturnType, typename... Args>
        bool wait_for(UniqueLock &lock,
                      const std::chrono::duration<Rep, Period> &timeout_duration,
                      ReturnType (*pred)(Args...), Args... args)
        {
            if (pred(std::forward<Args>(args)...))
            {
                return true;
            }
            return wait_for(lock, timeout_duration);
        }

        template <typename Rep, typename Period, typename ReturnType, typename T, typename... Args>
        bool wait_for(UniqueLock &lock,
                      const std::chrono::duration<Rep, Period> &timeout_duration,
                      ReturnType (T::*pred)(Args...), T *obj, Args &&...args)
        {
            if (obj->pred(std::forward<Args>(args)...))
            {
                return true;
            }
            return wait_for(lock, timeout_duration);
        }

        template <typename Rep, typename Period, typename Func, typename... Args>
        bool wait_for(UniqueLock &lock,
                      const std::chrono::duration<Rep, Period> &timeout_duration,
                      Func &&func, Args &&...args)
        {
            if (pred(std::forward<Args>(args)...))
            {
                return true;
            }
            return wait_for(lock, timeout_duration);
        }

        void wait(Mutex &mtx)
        {
#if UTOOLS_THREAD_FUNCATION == UTOOLS_STD_THREAD_FUNCATION
            std::unique_lock<std::mutex> lock(static_cast<std::mutex &>(mtx)); // 将 Mutex 转换为 std::mutex
            cond_var.wait(lock);                                               // 等待条件
#elif UTOOLS_THREAD_FUNCATION == UTOOLS_PTHREAD_FUNCATION
            pthread_mutex_unlock(&mutex);     // 解锁 Mutex
            pthread_cond_wait(&cond, &mutex); // 等待条件
            pthread_mutex_lock(&mutex);       // 再次锁定 Mutex
#elif UTOOLS_THREAD_FUNCATION == UTOOLS_FREERTOS_TASK_FUNCATION
            mtx.unlock();                             // 先解锁 Mutex
            xSemaphoreTake(semaphore, portMAX_DELAY); // 等待条件
            mtx.lock();                               // 再次锁定 Mutex
#endif
        }

        template <typename ReturnType, typename... Args>
        void wait(Mutex &mtx, ReturnType (*pred)(Args...), Args... args)
        {
            if (pred(std::forward<Args>(args)...))
            {
                return;
            }
            wait(mtx);
        }

        template <typename ReturnType, typename T, typename... Args>
        void wait(Mutex &mtx, ReturnType (T::*pred)(Args...), T *obj, Args &&...args)
        {
            if (obj->pred(std::forward<Args>(args)...))
            {
                return;
            }
            wait(mtx);
        }

        template <typename Func, typename... Args>
        void wait(Mutex &mtx, Func &&pred, Args &&...args)
        {
            if (pred(std::forward<Args>(args)...))
            {
                return;
            }
            wait(mtx);
        }

        /// @brief 等待条件变量，直到超时或条件满足
        /// @param mtx 互斥锁
        /// @param timeout_duration 超时时间
        /// @return true 表示条件满足，false 表示超时
        template <typename Rep, typename Period>
        bool wait_for(Mutex &mtx, const std::chrono::duration<Rep, Period> &timeout_duration)
        {
#if UTOOLS_THREAD_FUNCATION == UTOOLS_STD_THREAD_FUNCATION
            std::unique_lock<std::mutex> lock(static_cast<std::mutex &>(mtx));
            return cond_var.wait_for(lock, timeout_duration) == std::cv_status::no_timeout; // 返回是否成功
#elif UTOOLS_THREAD_FUNCATION == UTOOLS_PTHREAD_FUNCATION
            int millseconds = std::chrono::duration_cast<std::chrono::milliseconds>(timeout_duration).count();
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_sec += milliseconds / 1000;
            ts.tv_nsec += (milliseconds % 1000) * 1000000;

            pthread_mutex_unlock(&mutex);
            int result = pthread_cond_timedwait(&cond, &mutex, &ts);
            pthread_mutex_lock(&mutex);
            return (result == 0); // 返回是否成功
#elif UTOOLS_THREAD_FUNCATION == UTOOLS_FREERTOS_TASK_FUNCATION
            TickType_t timeout = std::chrono::duration_cast<std::chrono::milliseconds>(timeout_duration).count();
            mtx.unlock();                                     // 解锁 Mutex
            bool result = xSemaphoreTake(semaphore, timeout); // 等待条件，带超时
            mtx.lock();                                       // 再次锁定 Mutex
            return result;
#endif
            return false; // 如果没有成功等待，则返回 false
        }

        template <typename Rep, typename Period, typename ReturnType, typename... Args>
        bool wait_for(Mutex &mtx,
                      const std::chrono::duration<Rep, Period> &timeout_duration,
                      ReturnType (*pred)(Args...), Args... args)
        {
            if (pred(std::forward<Args>(args)...))
            {
                return true;
            }
            return wait_for(mtx, timeout_duration);
        }

        template <typename Rep, typename Period, typename ReturnType, typename T, typename... Args>
        bool wait_for(Mutex &mtx,
                      const std::chrono::duration<Rep, Period> &timeout_duration,
                      ReturnType (T::*pred)(Args...), T *obj, Args &&...args)
        {
            if (obj->pred(std::forward<Args>(args)...))
            {
                return true;
            }
            return wait_for(mtx, timeout_duration);
        }

        template <typename Rep, typename Period, typename Func, typename... Args>
        bool wait_for(Mutex &mtx,
                      const std::chrono::duration<Rep, Period> &timeout_duration,
                      Func &&func, Args &&...args)
        {
            if (pred(std::forward<Args>(args)...))
            {
                return true;
            }
            return wait_for(mtx, timeout_duration);
        }

        // #if UTOOLS_THREAD_FUNCATION == UTOOLS_STD_THREAD_FUNCATION
        //         template <typename Rep, typename Period>
        //         bool wait_for(Mutex &mtx, const std::chrono::duration<Rep, Period> &timeout_duration)
        //         {
        //             std::unique_lock<std::mutex> lock(static_cast<std::mutex &>(mtx));
        //             return cond_var.wait_for(lock, timeout_duration) == std::cv_status::no_timeout; // 返回是否成功
        //         }
        // #elif UTOOLS_THREAD_FUNCATION == UTOOLS_PTHREAD_FUNCATION
        //         bool wait_for(Mutex &mtx, int milliseconds)
        //         {
        //             struct timespec ts;
        //             clock_gettime(CLOCK_REALTIME, &ts);
        //             ts.tv_sec += milliseconds / 1000;
        //             ts.tv_nsec += (milliseconds % 1000) * 1000000;

        //             pthread_mutex_unlock(&mutex);
        //             int result = pthread_cond_timedwait(&cond, &mutex, &ts);
        //             pthread_mutex_lock(&mutex);
        //             return (result == 0); // 返回是否成功
        //         }
        // #elif UTOOLS_THREAD_FUNCATION == UTOOLS_FREERTOS_TASK_FUNCATION
        //         bool wait_for(Mutex &mtx, TickType_t timeout)
        //         {
        //             mtx.unlock();                                     // 解锁 Mutex
        //             bool result = xSemaphoreTake(semaphore, timeout); // 等待条件，带超时
        //             mtx.lock();                                       // 再次锁定 Mutex
        //             return result;                                    // 返回是否成功
        //         }
        // #endif

        void notify_one()
        {
#if UTOOLS_THREAD_FUNCATION == UTOOLS_STD_THREAD_FUNCATION
            cond_var.notify_one(); // 唤醒一个等待的线程
#elif UTOOLS_THREAD_FUNCATION == UTOOLS_PTHREAD_FUNCATION
            pthread_cond_signal(&cond); // 唤醒一个等待的线程
#elif UTOOLS_THREAD_FUNCATION == UTOOLS_FREERTOS_TASK_FUNCATION
            xSemaphoreGive(semaphore); // 唤醒一个等待的线程
#endif
        }

        void notify_all()
        {
#if UTOOLS_THREAD_FUNCATION == UTOOLS_STD_THREAD_FUNCATION
            cond_var.notify_all(); // 唤醒所有等待的线程
#elif UTOOLS_THREAD_FUNCATION == UTOOLS_PTHREAD_FUNCATION
            pthread_cond_broadcast(&cond); // 唤醒所有等待的线程
#elif UTOOLS_THREAD_FUNCATION == UTOOLS_FREERTOS_TASK_FUNCATION
            for (int i = 0; i < configMAX_PRIORITIES; ++i)
            {
                xSemaphoreGive(semaphore); // 唤醒所有等待的线程
            }
#endif
        }

    private:
#if UTOOLS_THREAD_FUNCATION == UTOOLS_STD_THREAD_FUNCATION
        std::condition_variable cond_var;
#elif UTOOLS_THREAD_FUNCATION == UTOOLS_PTHREAD_FUNCATION
        pthread_cond_t cond;
        pthread_mutex_t mutex;
#elif UTOOLS_THREAD_FUNCATION == UTOOLS_FREERTOS_TASK_FUNCATION
        SemaphoreHandle_t semaphore;
#endif
    };
}

#endif // __UTOOLS_UTCONDITION_VARIABLE_H__