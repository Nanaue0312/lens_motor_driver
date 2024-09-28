/// @brief 观察者模式

#ifndef __UTOOLS_PATTERN_OBSERVE_H__
#define __UTOOLS_PATTERN_OBSERVE_H__

#include <vector>
#include <algorithm>

namespace utools::pattern::observe
{
    /// @brief 观察者接口
    /// @tparam T 传输的数据类型
    template <typename T>
    class Observer
    {
    public:
        virtual ~Observer() {}

        /// @brief 更新方法
        /// @param data 传输的数据
        /// @warning 调用此方法时，不要修改data，如果使用的场景中有很多个观察者，需要处理可能出现阻赛情况
        virtual void update(const T &data) = 0;
    };

    /// @brief 被观察者基类
    /// @tparam T 传输的数据类型
    template <typename T>
    class Subject
    {
    public:
        virtual ~Subject() {}

        /// @brief 增加一个观察者
        /// @param observer 观察者对象
        /// @return Subject<T> &自身引用
        Subject<T> &attach(Observer<T> *observer)
        {
            observers.push_back(observer);
            return *this; // 返回自身引用
        }

        /// @brief 删除一个观察者
        /// @param observer 观察者对象
        /// @return Subject<T> &自身引用
        Subject<T> &detach(Observer<T> *observer)
        {
            observers.erase(std::remove(observers.begin(), observers.end(), observer), observers.end());
            return *this; // 返回自身引用
        }

    protected:
        /// @brief 提供给子类使用,用于更新推给所有观察者
        /// @param data 需要进行通知的数据
        void notify(const T &data)
        {
            for (const auto &observer : observers)
            {
                observer->update(data);
            }
        }

    private:
        std::vector<Observer<T> *> observers; // 观察者列表
    };
}

#endif