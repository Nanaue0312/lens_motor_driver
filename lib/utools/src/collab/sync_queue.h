/// @brief 提供一个线程安全的队列，用于同步数据
///        生产者线程使用push()方法将数据放入队列，消费者线程使用pop()方法从队列中获取数据
///        队列的大小由DATA_COUNT参数指定，如果DATA_COUNT为0，则不限制队列大小
///        如果队列满了，生产者线程会阻塞，直到消费者线程从队列中取出数据
///        如果队列为空，消费者线程会阻塞，直到生产者线程将数据放入队列

#ifndef __SYNC_QUEUE_H__
#define __SYNC_QUEUE_H__

#if UTOOLS_COLLAB_SYNC_QUEUE_ENABLE == 1

#include <queue>
#include <functional>
#include "utcondition_variable.h"
#include "utmutex.h"

namespace utools::collab
{
    /// @brief 同步队列
    /// @tparam _SYNC_DATA_TYPE 对列中的数据类型
    /// @tparam DATA_COUNT 最大队列大小，如果为0，则不限制队列大小
    template <typename _SYNC_DATA_TYPE, size_t DATA_COUNT = 20>
    class SyncQueue
    {
    public:
        using size_type = typename std::queue<_SYNC_DATA_TYPE>::size_type;
        using value_type = typename std::queue<_SYNC_DATA_TYPE>::value_type;
        using reference = typename std::queue<_SYNC_DATA_TYPE>::reference;
        using const_reference = typename std::queue<_SYNC_DATA_TYPE>::const_reference;

        SyncQueue() = default;
        virtual ~SyncQueue() {}

        /// @brief 向对列中增加一个数据
        /// @param data 要增加的数据
        void push(const value_type &data)
        {
            {
                ::utools::collab::UniqueLock lock(__mutex);
                __dealwith_queue_count_limit();
                __queue.push(data);
            }
            __cv.notify_one();
        }

        /// @brief 向对列中增加一个数据
        /// @param data 要增加的数据
        void push(value_type &&data)
        {
            ::utools::collab::UniqueLock lock(__mutex);
            __dealwith_queue_count_limit();
            __queue.push(std::move(data));
            __cv.notify_one();
        }

        /// @brief 向对列中增加一个数据，会直接调用构造函数
        /// @param args 要增加的数据的变量
        template <typename... Args>
        void emplace(Args &&...args)
        {
            {
                ::utools::collab::UniqueLock lock(__mutex);
                __dealwith_queue_count_limit();
                __queue.emplace(std::forward<Args>(args)...);
            }
            __cv.notify_one();
        }

        /// @brief 等待数据弹出，会返回数据
        /// @return 头部数据
        value_type pop_wait()
        {
            ::utools::collab::UniqueLock lock(__mutex);
            while (__queue.empty() || __released)
            {
                __cv.wait(lock);
            }
            auto data{std::move(__queue.front())};
            __queue.pop();
            return data;
        }

        /// @brief 等待数据弹出，会返回数据
        /// @param data 返回的数据
        void pop_wait(value_type &data)
        {
            ::utools::collab::UniqueLock lock(__mutex);
            while (__queue.empty() || __released)
            {
                __cv.wait(lock);
            }
            data = std::move(__queue.front());
            __queue.pop();
        }

        /// @brief 等待数据非空时
        /// @return true: 队列非空，false: 队列已释放
        bool wait_not_empty()
        {
            ::utools::collab::UniqueLock lock(__mutex);
            while (__queue.empty() || __released)
            {
                __cv.wait(lock);
            }
            return __released == false;
        }

        /// @brief 只弹出数据，不会返回任何数据
        void pop()
        {
            ::utools::collab::UniqueLock lock(__mutex);
            __queue.pop();
        }

        /// @brief 获取对列中的数据
        /// @return 头部分的数据
        const_reference front() const
        {
            ::utools::collab::UniqueLock lock(__mutex);
            return __queue.front();
        }

        reference front()
        {
            ::utools::collab::UniqueLock lock(__mutex);
            return __queue.front();
        }

        const_reference back() const
        {
            ::utools::collab::UniqueLock lock(__mutex);
            return __queue.back();
        }

        reference back()
        {
            ::utools::collab::UniqueLock lock(__mutex);
            return __queue.back();
        }

        size_type size() const
        {
            ::utools::collab::UniqueLock lock(__mutex);
            return __queue.size();
        }

        bool empty() const
        {
            ::utools::collab::UniqueLock lock(__mutex);
            return __queue.empty();
        }

        /// @brief 发送通知释放当前的pop()操作
        /// @warning 如不了解工作过程，请不要手动调用该函数
        void notify_one()
        {
            __cv.notify_one();
        }

        /// @brief 发送通知释放当前的pop()操作
        /// @warning 如不了解工作过程，请不要手动调用该函数
        void notify_all()
        {
            __cv.notify_all();
        }

        void release()
        {
            __released = true;
            notify_all();
        }

        /// @brief 重新初始化对列
        void reset()
        {
            __released = true;
            notify_all();
            ::utools::collab::UniqueLock lock(__mutex);
            __queue = std::queue<_SYNC_DATA_TYPE>();
            __released = false;
        }

        /// @brief 获取保存在对列中的数据
        /// @return 返回保存在对列中的数据
        /// @warning 如果对数据进行读写操作，可能会引发异常
        std::queue<_SYNC_DATA_TYPE> &data() const
        {
            return __queue;
        }

    private:
        ::utools::collab::Mutex __mutex;
        ::utools::collab::ConditionVariable __cv;
        std::queue<_SYNC_DATA_TYPE> __queue;
        bool __released{false};

        /// @brief 检查是否已经达到队列上限
        inline void __dealwith_queue_count_limit()
        {
            if (DATA_COUNT > 0 && __queue.size() >= DATA_COUNT)
            {
                __queue.pop();
            }
        }
    };

    template <typename _SYNC_DATA_TYPE, size_t DATA_COUNT = 20>
    SyncQueue<_SYNC_DATA_TYPE, DATA_COUNT> make_sync_queue()
    {
        return SyncQueue<_SYNC_DATA_TYPE, DATA_COUNT>();
    }
}

#endif // UTOOLS_COLLAB_SYNC_QUEUE_ENABLE
#endif // __SYNC_QUEUE_H__
