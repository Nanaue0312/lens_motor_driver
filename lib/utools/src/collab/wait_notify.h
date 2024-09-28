#ifndef __WAIT_NOTIFY_H__
#define __WAIT_NOTIFY_H__

#if UTOOLS_COLLAB_WAIT_NOTIFY_ENABLE == 1

#include <thread>
#include <chrono>
#include <functional>
#include <condition_variable>
#include <mutex>

namespace utools::collab
{
    template <typename Duration, typename FUN, typename... ARGS>
    bool wait_for(FUN &&fun, Duration timeout, ARGS &&...args)
    {
        std::condition_variable cv;
        std::mutex mtx;
        bool ready = false;
        bool timeout_occurred = false;

        std::thread([&]()
                    {
        std::unique_lock<std::mutex> lock(mtx);
        ready = fun(std::forward<ARGS>(args)...);
        cv.notify_one(); })
            .detach();

        std::unique_lock<std::mutex> lock(mtx);
        if (!cv.wait_for(lock, timeout, [&]()
                         { return ready; }))
        {
            timeout_occurred = true;
        }

        return !timeout_occurred;
    }

    template <typename FUN, typename... ARGS>
    bool wait_for_ms(FUN &&fun, int timeout, ARGS &&...args)
    {
        return wait_for(std::forward<FUN>(fun),
                        std::chrono::milliseconds(timeout),
                        std::forward<ARGS>(args)...);
    }
}

#endif // UTOOLS_COLLAB_WAIT_NOTIFY_ENABLE
#endif // __WAIT_NOTIFY_H__