#ifndef __UTOOLS_DISPATCHER_H__
#define __UTOOLS_DISPATCHER_H__

#include <stdint.h>
#include <functional>
#include <unordered_map>
#include <memory>

namespace utools::collab
{
    template <typename KeyT, typename FuncT>
    class Dispatcher;

    template <typename KeyT, typename ReturnType, typename... Args>
    class Dispatcher<KeyT, ReturnType(Args...)>
    {
    public:
        using WorkFunctionType = std::function<ReturnType(Args...)>;

        Dispatcher() = default;
        virtual ~Dispatcher() = default;

        /// @brief 设置映射调度任务 - 接收任意可调用对象
        /// @param func 可调用对象
        /// @param args 可变参数
        /// @return 返回任务对象本身
        template <typename Func>
        Dispatcher &map(const KeyT key, Func &&func)
        {
            __dispatcher_map[key] = WorkFunctionType(std::forward<Func>(func));
            return *this;
        }

        /// @brief 设置映射调度任务 - 支持成员函数
        /// @param func 类成员函数
        /// @param obj 类对象指针
        /// @param args 可变参数
        /// @return 返回任务对象本身
        template <typename T, typename MemberFunc>
        Dispatcher &map(const KeyT key, MemberFunc &&func, T *obj)
        {
            __dispatcher_map[key] = [obj, func](Args... args)
            {
                return (obj->*func)(std::forward<Args>(args)...); // 调用成员函数
            };
            return *this;
        }

        /// @brief 删除映射调度任务
        /// @return 返回自身
        Dispatcher &unmap(const KeyT key)
        {
            __dispatcher_map.erase(key);
            return *this;
        }

        /// @brief 调度任务
        /// @param key 调度任务标识
        /// @param args 可变参数
        ReturnType dispatch(const KeyT key, Args... args)
        {
            auto it = __dispatcher_map.find(key);
            if (it != __dispatcher_map.end())
            {
                return it->second(std::forward<Args>(args)...);
            }
        }

    private:
        std::unordered_map<KeyT, WorkFunctionType> __dispatcher_map; // 存储调度任务
    };
}

#endif // __UTOOLS_DISPATCHER_H__