#ifndef __UTOOLS_MACRO_H__
#define __UTOOLS_MACRO_H__

// 辅助宏，用于将 __LINE__ 转化为字符串
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

// 辅助宏，用于检查可变参数是否为空
#define CHECK_VA_ARGS_EMPTY_IMPL(first, ...) (sizeof(#first) > 1)
#define CHECK_VA_ARGS_EMPTY(...) CHECK_VA_ARGS_EMPTY_IMPL(__VA_ARGS__)

/// @brief 定义一个宏来检查命名空间中的符号是否存在
/// @details 使用方法：
//          CHECK_NAMESPACE_SYMBOL(utools::math, add)
//          int main()
//          {
//              if (has_add<utools::math>::value)
//              {
//                  std::cout << "utools::math namespace exists and contains the 'add' function." << std::endl;
//              }
//              else
//              {
//                  std::cout << "utools::math namespace does not exist or does not contain the 'add' function." << std::endl;
//              }
//              return 0;
//          }
#define CHECK_NAMESPACE_SYMBOL(Namespace, Symbol)                                  \
    template <typename T>                                                          \
    struct has_##Symbol                                                            \
    {                                                                              \
        template <typename U>                                                      \
        static constexpr auto check(U *) ->                                        \
            typename std::is_same<decltype(U::Symbol), decltype(U::Symbol)>::type; \
        template <typename>                                                        \
        static constexpr std::false_type check(...);                               \
        using type = decltype(check<Namespace>(0));                                \
        static constexpr bool value = type::value;                                 \
    };                                                                             \
    static_assert(has_##Symbol<Namespace>::value, "Symbol not found in namespace");

#endif // __UTOOLS_MACRO_H__
