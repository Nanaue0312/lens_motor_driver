/// @brief 消费者生产者任务处理机制
///        将任务分为三个阶段，分别是输入、处理、输出
///        输入阶段：将数据放入队列
///        处理阶段：从队列中取出数据，进行处理
///        输出阶段：将处理后的数据输出
///        使用SyncQueue来实现队列的同步
///        使用std::thread来实现多线程处理

#ifndef __UTOOLS_COLLAB_STREAM_TASK_H__
#define __UTOOLS_COLLAB_STREAM_TASK_H__

#include "../utools_cfg.h"

#if UTOOLS_COLLAB_FLOWTASK_ENABLE == 1

#if UTOOLS_THREAD_FUNCATION == UTOOLS_THREAD_FUNCATION_NULL
#error "UTOOLS_THREAD_FUNCATION must be set to UTOOLS_THREAD_FUNCATION_STD or UTOOLS_THREAD_FUNCATION_FREERTOS or UTOOLS_PTHREAD_FUNCATION!"
#endif // UTOOLS_THREAD_FUNCATION_NULL

#include "sync_queue.h"
#include "collab_task.h"

#include <cstdint>
#include <type_traits>
#include <thread>
#include <functional>

namespace utools::collab
{
    /// @brief 流程式任务，用于处理流程式数据
    /// @tparam _SYNC_DATA_TYPE 对列中的数据类型
    /// @tparam USE_INPUTFUNC 是否使用输入函数，如果不使用，需要外部的输入函数
    /// @tparam USE_OUTPUTFUNC 是否使用输出函数
    /// @tparam DATA_COUNT 最大队列大小，如果为0，则不限制队列大小
    template <typename _SYNC_DATA_TYPE,
              bool USE_INPUTFUNC = true,
              bool USE_OUTPUTFUNC = true,
              std::size_t DATA_COUNT = 64>
    class FlowTask
    {
    public:
        using queue_type = SyncQueue<_SYNC_DATA_TYPE, DATA_COUNT>;
        using data_type = _SYNC_DATA_TYPE;
        using flow_task_type = FlowTask<_SYNC_DATA_TYPE, USE_INPUTFUNC, USE_OUTPUTFUNC, DATA_COUNT>;

        FlowTask() = default;

        virtual ~FlowTask()
        {
            if (USE_INPUTFUNC)
            {
                __consume_task.stop();
            }
            __produce_task.stop();
        }

        /// @brief 开始任务
        /// @return true表示成功，false表示失败
        bool start()
        {
            if (__running)
            {
                return false;
            }
            if (!(__input_func && __handle_func))
            {
                return false;
            }
            if (USE_OUTPUTFUNC && !__output_func)
            {
                return false;
            }

            __running = true;
            if (USE_INPUTFUNC)
            {
                __consume_task = Task(&flow_task_type::__consume_worker, this).start(1024 * 10);
            }
            __produce_task = Task(&flow_task_type::__produce_worker, this).start(1024 * 10);
            return true;
        }

        /// @brief 停止任务
        void stop()
        {
            __running = false;
            __queue.release();
        }

        void push(_SYNC_DATA_TYPE &data)
        {
            __queue.push(data);
        }

        void push(const _SYNC_DATA_TYPE &data)
        {
            __queue.push(data);
        }

        template <typename... Args>
        void emplace(Args &&...args)
        {
            __queue.emplace(std::forward<Args>(args)...);
        }

        /// @brief 绑定输入函数
        /// @param input_func bool(queue_type &)类型的函数对象，bool表示是否成功读取到信息，queue_type &表示队列，用户写入时使用此队列进行操作
        void bind_on_input(std::function<bool(queue_type &)> &input_func)
        {
            __input_func = input_func;
        }

        void bind_on_input(bool (*on_input_func)(queue_type &))
        {
            __input_func = on_input_func;
        }

        template <typename T>
        void bind_on_input(bool (T::*on_input_func)(queue_type &), T *obj)
        {
            __input_func = [on_input_func, obj](queue_type &queue) -> bool
            {
                return (obj->*on_input_func)(queue);
            };
        }

        /// @brief 绑定处理函数
        /// @param handle_func void(data_type &)类型的函数对象，data_type &表示数据，用户处理完数据后，使用此函数进行输出
        void bind_on_handler(std::function<bool(data_type &, void *)> &handle_func)
        {
            __handle_func = handle_func;
        }

        void bind_on_handler(bool (*handle_func)(data_type &, void *))
        {
            __handle_func = handle_func;
        }

        template <typename T>
        void bind_on_handler(bool (T::*handle_func)(data_type &, void *), T *obj)
        {
            __handle_func = [handle_func, obj](data_type &data, void *consumer_data) -> bool
            {
                return (obj->*handle_func)(data, consumer_data);
            };
        }

        /// @brief 绑定输出函数
        /// @param handle_func bool(data_type &, void *)类型的函数对象，bool表示是否成功处理数据，data_type &表示数据，void *用户数据，会传下去
        void bind_on_output(std::function<void(data_type &, void *)> &output_func)
        {
            __output_func = output_func;
        }

        void bind_on_output(void (*on_output_func)(data_type &, void *))
        {
            __output_func = on_output_func;
        }

        template <typename T>
        void bind_on_output(void (T::*on_output_func)(data_type &, void *), T *obj)
        {
            __output_func = [on_output_func, obj](data_type &data, void *consumer_data)
            {
                (obj->*on_output_func)(data, consumer_data);
            };
        }

        /// @brief 获取数据流的处理对列
        /// @return 处理对列
        queue_type &get_queue()
        {
            return __queue;
        }

    private:
        queue_type __queue;

        /// @brief  消费线程读取数据的接口
        /// @param queue_type 处理对列，消费者可以通过此对写入数据
        std::function<bool(queue_type &)> __input_func;

        /// @brief  生产线程处理数据的接口
        /// @param data_type (data)处理的数据
        /// @param void *(user_data)用户数据，可以传给handle_func和output_func
        std::function<bool(data_type &, void *)> __handle_func;

        /// @brief  生产线程输出数据的接口
        /// @param data_type (data)处理的数据
        /// @param void *(user_data)用户数据，可以传给handle_func和output_func
        std::function<void(data_type &, void *)> __output_func;

        Task __consume_task;
        Task __produce_task;

        bool __running{false};

        void __consume_worker()
        {
            while (__running)
            {
                __input_func(__queue); // NOTE：生产者线程，用于将数据写入到队列中，写入后会自动通知处理
            }
        }

        void __produce_worker()
        {
            while (__running)
            {
                auto strame_data{__queue.pop_wait()};
                void *consumer_data{nullptr}; // NOTE: 消费者数据，可以传给handle_func和output_func
                if (__handle_func(strame_data, consumer_data))
                {
                    if (USE_OUTPUTFUNC)
                    {
                        __output_func(strame_data, consumer_data);
                    }
                }
            }
        }
    };
}

#endif // UTOOLS_COLLAB_FLOWTASK_ENABLE

#endif // __UTOOLS_COLLAB_STREAM_TASK_H__
