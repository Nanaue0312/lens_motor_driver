#ifndef __UTIMER_H__
#define __UTIMER_H__

#include <thread>
#include <chrono>
#include <mutex>
#include <functional>
#include <atomic>
#include "../utools_cfg.h"

namespace utools::time
{
#if UTOOLS_TIMER_ENABLE == 1
    class Timer
    {
    public:
        Timer() : running_(false), interval_(std::chrono::milliseconds(1000)) {}

        ~Timer()
        {
            stop();
        }

        void start()
        {
            if (running_ || !callback_)
                return;

            running_ = true;
            worker_thread_ = std::thread(std::bind(&Timer::worker, this));
        }

        void stop()
        {
            running_ = false;
            if (worker_thread_.joinable())
            {
                worker_thread_.join();
            }
        }

        void set_interval(std::chrono::milliseconds interval)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            interval_ = interval;
        }

        void set_callback(std::function<void()> callback)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            callback_ = callback;
        }

    private:
        std::atomic<bool> running_;
        std::chrono::milliseconds interval_;
        std::function<void()> callback_{nullptr};
        std::thread worker_thread_;
        std::mutex mutex_;

        void worker()
        {
            while (running_)
            {
                auto start_time = std::chrono::steady_clock::now();
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    callback_();
                }
                auto end_time = std::chrono::steady_clock::now();
                std::this_thread::sleep_for(interval_ - (end_time - start_time));
            }
        }
    };
#endif // UTOOLS_TIMER_ENABLE
}

#endif // __UTIMER_H__