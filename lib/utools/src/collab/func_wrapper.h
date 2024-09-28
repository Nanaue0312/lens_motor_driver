/// @brief 函数包装器，用于将函数指针转换为函数对象
///        提供需要必须调用c函数的场景，将stl或类中的函数打包可c类型的函数指针

#ifndef __FUNC_WRAPPER_H__
#define __FUNC_WRAPPER_H__

#if UTOOLS_COLLAB_FUNC_WRAPPER_ENABLE == 1

#include <functional>
#include <memory>
#include <type_traits>
#include <utility>

namespace utools::collab
{
    template <typename ReturnType, typename... Args>
    class CFuncWrapper
    {
    public:
        using FuncType = std::function<ReturnType(Args...)>;
        using CFuncType = ReturnType (*)(Args...);

        CFuncWrapper(FuncType func)
            : func_(func) {}

        static CFuncType c_func(std::shared_ptr<CFuncWrapper> wrapper)
        {
            instance = wrapper; // 保持shared_ptr的引用计数
            return [](Args... args) -> ReturnType
            {
                return instance->func_(args...);
            };
        }

    private:
        FuncType func_;
        static std::shared_ptr<CFuncWrapper> instance; // 静态智能指针实例
    };

    // 定义静态成员变量
    template <typename ReturnType, typename... Args>
    std::shared_ptr<CFuncWrapper<ReturnType, Args...>> CFuncWrapper<ReturnType, Args...>::instance = nullptr;

    /// @brief 将成员函数指针转换为C风格的函数指针
    /// @param func 成员函数指针
    /// @param obj 指向成员函数所属对象的指针
    /// @return C风格的函数指针
    template <typename ReturnType, typename T, typename... Args>
    auto to_c_func(ReturnType (T::*func)(Args...), T *obj)
        -> typename CFuncWrapper<ReturnType, Args...>::CFuncType
    {
        using Wrapper = CFuncWrapper<ReturnType, Args...>;

        auto bound_func = [func, obj](Args... args) -> ReturnType
        {
            return (obj->*func)(std::forward<Args>(args)...);
        };

        auto wrapper = std::make_shared<Wrapper>(bound_func);
        return Wrapper::c_func(wrapper);
    }

    /// @brief 将普通函数指针转换为C风格的函数指针
    /// @tparam ReturnType 返回值类型
    /// @tparam ...Args 参数类型
    /// @param func 函数指针
    /// @return C风格的函数指针
    template <typename ReturnType, typename... Args>
    auto to_c_func(ReturnType (*func)(Args...))
        -> typename CFuncWrapper<ReturnType, Args...>::CFuncType
    {
        using Wrapper = CFuncWrapper<ReturnType, Args...>;

        auto wrapper = std::make_shared<Wrapper>(func);
        return Wrapper::c_func(wrapper);
    }

    /// @brief 将可调用对象转换为C风格的函数指针
    /// @param func 普通函数
    /// @return C风格的函数指针
    template <typename Func, typename... Args>
    auto to_c_func(Func func)
        -> typename CFuncWrapper<typename std::result_of<Func>::type, Args...>::CFuncType
    {
        using Wrapper = CFuncWrapper<typename std::result_of<Func>::type, Args...>;

        auto wrapper = std::make_shared<Wrapper>(func);
        return Wrapper::c_func(wrapper);
    }

    // 自定义索引序列工具
    template <std::size_t... Is>
    struct index_sequence
    {
    };

    template <std::size_t N, std::size_t... Is>
    struct make_index_sequence : make_index_sequence<N - 1, N - 1, Is...>
    {
    };

    template <std::size_t... Is>
    struct make_index_sequence<0, Is...> : index_sequence<Is...>
    {
    };

    // 辅助模板类：提取前 N 个参数
    template <typename Tuple, std::size_t... Is>
    auto extract_first_n_helper(Tuple &&t, index_sequence<Is...>)
        -> decltype(std::make_tuple(std::get<Is>(std::forward<Tuple>(t))...))
    {
        return std::make_tuple(std::get<Is>(std::forward<Tuple>(t))...);
    }

    template <std::size_t N, typename Tuple>
    auto extract_first_n(Tuple &&t)
        -> decltype(extract_first_n_helper(std::forward<Tuple>(t), make_index_sequence<N>{}))
    {
        return extract_first_n_helper(std::forward<Tuple>(t), make_index_sequence<N>{});
    }

    // 获取返回类型
    template <typename T>
    struct function_traits
    {
        using return_type = void;
        using parameter_types = void;
    };

    template <typename ReturnType, typename... Args>
    struct function_traits<ReturnType (*)(Args...)>
    {
        using return_type = ReturnType;
        using parameter_types = std::tuple<Args...>;
    };

    template <typename ReturnType, typename T, typename... Args>
    struct function_traits<ReturnType (T::*)(Args...)>
    {
        using return_type = ReturnType;
        using parameter_types = std::tuple<Args...>;
    };

    template <typename T>
    struct is_function_pointer : std::false_type
    {
    };

    template <typename ReturnType, typename... Args>
    struct is_function_pointer<ReturnType (*)(Args...)> : std::true_type
    {
    };

    // FIXME：此函数有不能使用
    // template <typename ReturnType, typename... Args, typename RunFunc, typename... RArgs>
    // auto extend_c_func_param_impl(ReturnType (*func)(std::tuple<Args...>),
    //                               CFuncWrapper<RunFunc, RArgs...> *wrapper)
    //     -> typename CFuncWrapper<ReturnType, Args...>::CFuncType
    // {
    //     using Wrapper = CFuncWrapper<ReturnType, Args...>;
    //     auto first_n_args = extract_first_n<sizeof...(RArgs)>(std::make_tuple(args...));

    //     auto bound_func = [func, wrapper](Args... args) -> ReturnType
    //     {
    //         if (std::is_same<ReturnType, void>::value)
    //         {
    //             wrapper->c_func(std::forward<Args>(args)...) return;
    //         }
    //         else if (std::is_same<RunFunc, ReturnType>::value)
    //         {
    //             return wrapper->c_func(std::forward<Args>(args)...)
    //         }
    //         else
    //         {
    //             wrapper->c_func(std::forward<Args>(args)...);
    //             return ReturnType(); // 返回对应功能的返回值类型的默认构造函数的返回值
    //         }
    //     };
    //     auto wrapper = std::make_shared<Wrapper>(func);
    //     return Wrapper::c_func(wrapper);
    // }

    // /// @brief 将无参数的成员函数转换成c风格的函数指针，同时按照模板的要求，生成相应的调用函数
    // /// @note 此函数没有完成，请不要使用
    // template <typename TargetFun, typename ReturnType, typename... Args, typename T>
    // auto extend_c_func_param(ReturnType (T::*func)(Args...), T *obj)
    //     -> typename CFuncWrapper<typename function_traits<TargetFun>::return_type,
    //                              typename function_traits<TargetFun>::parameter_types>::CFuncType
    // {
    //     static_assert(!is_function_pointer<TargetFun>::value, "TargetFun must be a function pointer");
    //     using FuncReturnType = typename function_traits<TargetFun>::return_type;
    //     using ParameterTypes = typename function_traits<TargetFun>::parameter_types;
    //     using Wrapper = CFuncWrapper<FuncReturnType, ParameterTypes>;

    //     // 将数据包转换成tuple

    //     // FIXME: 此处参数的展开还未实现
    //     auto bound_func = [func, obj](Args args...) -> ReturnType
    //     {
    //         return (obj->*func)(std::forward<Args>(args)...);
    //     };

    //     auto wrapper = std::make_shared<Wrapper>(bound_func);
    //     extend_c_func_param_impl();

    //     return Wrapper::c_func(wrapper);
    // }
}

#endif // UTOOLS_COLLAB_FUNC_WRAPPER_ENABLE
#endif // __FUNC_WRAPPER_H__
