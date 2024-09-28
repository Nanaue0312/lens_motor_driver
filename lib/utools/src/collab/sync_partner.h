#ifndef __SYNC_PARTNER_H__
#define __SYNC_PARTNER_H__

#if UTOOLS_COLLAB_SYNC_PARTNER_ENABLE == 1

#include <mutex>
#include <condition_variable>
#include <thread>
#include <functional>

namespace utools::collab
{
    class SyncPartner
    {
    public:
        SyncPartner() = default;

        virtual ~SyncPartner() = default;

        /// @brief 在同步状态下，通过func来进行操作
        /// @tparam ReturnType 返回值类型
        /// @tparam T 对象类型
        /// @tparam ...Args 参数类型
        /// @param func 成员函数
        /// @param obj 类对象
        /// @param ...args 参数
        /// @return 返回值
        template <typename ReturnType, typename T, typename... Args>
        ReturnType notify_with(ReturnType (T::*func)(Args...), T *obj, Args... args)
        {
            std::unique_lock<std::mutex> lock(__mutex);
            if (std::is_same<void, ReturnType>::value)
            {
                (obj->*func)(std::forward<Args>(args)...);
                __condition.notify_one();
            }
            ReturnType result = (obj->*func)(std::forward<Args>(args)...);
            __condition.notify_one();
            return result;
        }

        /// @brief 在同步状态下，通过func来进行操作
        /// @tparam ReturnType 返回值类型
        /// @tparam ...Args 参数类型
        /// @param func 函数
        /// @param ...args 参数
        /// @return 返回值
        template <typename ReturnType, typename... Args>
        ReturnType notify_with(ReturnType (*func)(Args...), Args... args)
        {
            std::unique_lock<std::mutex> lock(__mutex);
            if (std::is_same<void, ReturnType>::value)
            {
                func(std::forward<Args>(args)...);
                __condition.notify_one();
            }
            ReturnType result = func(std::forward<Args>(args)...);
            __condition.notify_one();
            return result;
        }

        /// @brief 在同步状态下，等待pred为true，此函数执行完成后，会释放锁
        /// @tparam ReturnType 返回值类型
        /// @tparam T 对象类型
        /// @tparam ...Args 参数类型
        /// @param pred 成员函数
        /// @param obj 类对象
        /// @param ...args 参数
        /// @return 无
        template <typename ReturnType, typename T, typename... Args,
                  typename = std::enable_if<std::is_arithmetic<ReturnType>::value ||
                                            std::is_same<ReturnType, bool>::value ||
                                            std::is_pointer<ReturnType>::value>>
        void wait_with(ReturnType (T::*pred)(Args...), T *obj, Args... args)
        {
            std::unique_lock<std::mutex> lock(__mutex);
            while (!(obj->*pred)(std::forward<Args>(args)...))
            {
                __condition.wait(lock);
            }
        }

        /// @brief 在同步状态下，等待pred为true，此函数执行完成后，会释放锁
        /// @tparam ReturnType 返回值类型
        /// @tparam T 对象类型
        /// @tparam ...Args 参数类型
        /// @param pred 成员函数
        /// @param obj 类对象
        /// @param ...args 参数
        /// @return 无
        template <typename ReturnType, typename T, typename... Args,
                  typename = std::enable_if<std::is_arithmetic<ReturnType>::value ||
                                            std::is_same<ReturnType, bool>::value ||
                                            std::is_pointer<ReturnType>::value>>
        void wait_with(ReturnType (T::*pred)(Args...) const, T *obj, Args... args)
        {
            std::unique_lock<std::mutex> lock(__mutex);
            while (!(obj->*pred)(std::forward<Args>(args)...))
            {
                __condition.wait(lock);
            }
        }

        /// @brief 在同步状态下，等待pred为true，此函数执行完成后，会释放锁
        /// @tparam ReturnType 返回值类型
        /// @tparam ...Args 参数类型
        /// @param pred 条件函数
        /// @param ...args 参数
        /// @return 无
        template <typename ReturnType, typename... Args,
                  typename = std::enable_if<std::is_arithmetic<ReturnType>::value ||
                                            std::is_same<ReturnType, bool>::value ||
                                            std::is_pointer<ReturnType>::value>>
        void wait_with(ReturnType (*pred)(Args...), Args... args)
        {
            std::unique_lock<std::mutex> lock(__mutex);
            while (!pred(std::forward<Args>(args)...))
            {
                __condition.wait(lock);
            }
        }

        struct WaitGuard
        {
            std::mutex &mtx;
            bool owns_lock;

            explicit WaitGuard(std::mutex &mutex)
                : mtx(mutex), owns_lock(true)
            {
                mtx.lock();
            }

            ~WaitGuard()
            {
                if (owns_lock)
                {
                    mtx.unlock();
                }
            }

            // 禁止拷贝构造
            WaitGuard(const WaitGuard &) = delete;
            WaitGuard &operator=(const WaitGuard &) = delete;

            // 移动构造函数
            WaitGuard(WaitGuard &&other) noexcept
                : mtx(other.mtx), owns_lock(other.owns_lock)
            {
                other.owns_lock = false;
            }

            // 移动赋值操作符
            WaitGuard &operator=(WaitGuard &&other) noexcept
            {
                if (this != &other)
                {
                    if (owns_lock)
                    {
                        mtx.unlock();
                    }
                    // 直接转移所有权，没有赋值
                    owns_lock = other.owns_lock;
                    other.owns_lock = false;
                }
                return *this;
            }
        };

        /// @brief 在同步状态下，等待pred为true，此函数执行完成后，会返回锁，调用者出作用域后释放
        /// @tparam ReturnType 返回值类型
        /// @tparam T 对象类型
        /// @tparam ...Args 参数类型
        /// @param pred 成员函数
        /// @param obj 类对象
        /// @param ...args 参数
        /// @return 锁对象
        template <typename ReturnType, typename T, typename... Args,
                  typename = std::enable_if<std::is_arithmetic<ReturnType>::value ||
                                            std::is_same<ReturnType, bool>::value ||
                                            std::is_pointer<ReturnType>::value>>
        std::shared_ptr<std::unique_lock<std::mutex>> wait_guard(ReturnType (T::*pred)(Args...), T *obj, Args... args)
        {
            auto lock = std::make_shared<std::unique_lock<std::mutex>>(__mutex);
            while (!(obj->*pred)(std::forward<Args>(args)...))
            {
                __condition.wait(*lock);
            }
            return lock;
        }

        /// @brief 在同步状态下，等待pred为true，此函数执行完成后，会返回锁，调用者出作用域后释放
        /// @tparam ReturnType 返回值类型
        /// @tparam T 对象类型
        /// @tparam ...Args 参数类型
        /// @param pred 成员函数
        /// @param obj 类对象
        /// @param ...args 参数
        /// @return 锁对象
        template <typename ReturnType, typename T, typename... Args,
                  typename = std::enable_if<std::is_arithmetic<ReturnType>::value ||
                                            std::is_same<ReturnType, bool>::value ||
                                            std::is_pointer<ReturnType>::value>>
        std::shared_ptr<std::unique_lock<std::mutex>> wait_guard(ReturnType (T::*pred)(Args...) const, T *obj, Args... args)
        {
            auto lock = std::make_shared<std::unique_lock<std::mutex>>(__mutex);
            while (!(obj->*pred)(std::forward<Args>(args)...))
            {
                __condition.wait(*lock);
            }
            return lock;
        }

        /// @brief 在同步状态下，等待pred为true，此函数执行完成后，会返回锁，调用者出作用域后释放
        /// @tparam ReturnType 返回值类型
        /// @tparam ...Args 参数类型
        /// @param pred 条件函数
        /// @param ...args 参数
        /// @return 锁对象
        template <typename ReturnType, typename T, typename... Args,
                  typename = std::enable_if<std::is_arithmetic<ReturnType>::value ||
                                            std::is_same<ReturnType, bool>::value ||
                                            std::is_pointer<ReturnType>::value>>
        std::shared_ptr<std::unique_lock<std::mutex>> wait_guard(ReturnType (*pred)(Args...), Args... args)
        {
            auto lock = std::make_shared<std::unique_lock<std::mutex>>(__mutex);
            while (!pred(std::forward<Args>(args)...))
            {
                __condition.wait(*lock);
            }
            return lock;
        }

        /// @brief 在同步状态下，等待pred为true，此函数执行完成后，会返回锁，调用者出作用域后释放
        /// @tparam ReturnType 返回值类型
        /// @tparam ...Args 参数类型
        /// @param pred 条件函数
        /// @param ...args 参数
        /// @return 锁对象
        template <typename PRED_FUN, typename... Args,
                  typename = std::enable_if<std::is_arithmetic<std::result_of<PRED_FUN>>::value ||
                                            std::is_same<std::result_of<PRED_FUN>, bool>::value ||
                                            std::is_pointer<std::result_of<PRED_FUN>>::value>>
        std::shared_ptr<std::unique_lock<std::mutex>> wait_guard(PRED_FUN pred_fun, Args... args)
        {
            auto lock = std::make_shared<std::unique_lock<std::mutex>>(__mutex);
            while (!pred_fun(std::forward<Args>(args)...))
            {
                __condition.wait(*lock);
            }
            return lock;
        }

        /// @brief 通知等待的线程条件已满足
        void notify()
        {
            __condition.notify_one();
        }

    private:
        std::mutex __mutex;
        std::condition_variable __condition;
    };
}

#endif // UTOOLS_COLLAB_SYNC_PARTNER_ENABLE
#endif // __SYNC_PARTNER_H__