/// @brief 实现单例模式,用于创建全局唯一的对象,简化开发代码

#ifndef __UTOOLS_SINGLETON_H__
#define __UTOOLS_SINGLETON_H__

namespace utools::pattern
{
    template <typename T>
    class Singleton
    {
    public:
        // 禁用拷贝构造和赋值操作
        Singleton(const Singleton &) = delete;
        Singleton &operator=(const Singleton &) = delete;

        // 获取单例实例
        static T &instance()
        {
            static T instance; // 在第一次调用时创建实例
            return instance;
        }

    protected:
        Singleton() = default;  // 构造函数受保护
        ~Singleton() = default; // 析构函数受保护
    };
}

#endif // __UTOOLS_SINGLETON_H__
