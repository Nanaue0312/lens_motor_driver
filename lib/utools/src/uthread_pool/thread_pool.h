#ifndef __UTOOLS_THREAD_POOL_H__
#define __UTOOLS_THREAD_POOL_H__

#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <future>

namespace utools::pool
{
    // class ThreadPool
    // {
    // public:
    //     explicit ThreadPool(size_t threadCount) : stop(false)
    //     {
    //         for (size_t i = 0; i < threadCount; ++i)
    //         {
    //             workers.emplace_back([this]
    //                                  {
    //             while (true) {
    //                 std::function<void()> task;
    //                 {
    //                     std::unique_lock<std::mutex> lock(this->queueMutex);
    //                     this->condition.wait(lock, [this] {
    //                         return this->stop || !this->highPriorityTasks.empty() || !this->lowPriorityTasks.empty();
    //                     });

    //                     if (this->stop && this->highPriorityTasks.empty() && this->lowPriorityTasks.empty()) {
    //                         return;
    //                     }

    //                     if (!this->highPriorityTasks.empty()) {
    //                         task = std::move(this->highPriorityTasks.front());
    //                         this->highPriorityTasks.pop();
    //                     } else {
    //                         task = std::move(this->lowPriorityTasks.front());
    //                         this->lowPriorityTasks.pop();
    //                     }
    //                 }

    //                 task();
    //             } });
    //         }
    //     }

    //     template <typename F, typename... Args>
    //     auto enqueueImmediate(F &&f, Args &&...args) -> std::future<typename std::result_of<F(Args...)>::type>
    //     {
    //         using returnType = typename std::result_of<F(Args...)>::type;

    //         auto task = std::make_shared<std::packaged_task<returnType()>>(
    //             std::bind(std::forward<F>(f), std::forward<Args>(args)...));

    //         std::future<returnType> res = task->get_future();
    //         {
    //             std::unique_lock<std::mutex> lock(queueMutex);
    //             highPriorityTasks.emplace([task]()
    //                                       { (*task)(); });
    //         }
    //         condition.notify_one();
    //         return res;
    //     }

    //     template <typename F, typename... Args>
    //     auto enqueueLongTerm(F &&f, Args &&...args) -> std::future<typename std::result_of<F(Args...)>::type>
    //     {
    //         using returnType = typename std::result_of<F(Args...)>::type;

    //         auto task = std::make_shared<std::packaged_task<returnType()>>(
    //             std::bind(std::forward<F>(f), std::forward<Args>(args)...));

    //         std::future<returnType> res = task->get_future();
    //         {
    //             std::unique_lock<std::mutex> lock(queueMutex);
    //             lowPriorityTasks.emplace([task]()
    //                                      { (*task)(); });
    //         }
    //         condition.notify_one();
    //         return res;
    //     }

    //     ~ThreadPool()
    //     {
    //         {
    //             std::unique_lock<std::mutex> lock(queueMutex);
    //             stop = true;
    //         }
    //         condition.notify_all();
    //         for (std::thread &worker : workers)
    //         {
    //             worker.join();
    //         }
    //     }

    // private:
    //     std::vector<std::thread> workers;
    //     std::queue<std::function<void()>> highPriorityTasks;
    //     std::queue<std::function<void()>> lowPriorityTasks;
    //     std::mutex queueMutex;
    //     std::condition_variable condition;
    //     bool stop;
    // };
}

#endif // __UTOOLS_THREAD_POOL_H__
