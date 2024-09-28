#ifndef __UTOOLS_BYTE_PACKER_H__
#define __UTOOLS_BYTE_PACKER_H__

#include <vector>
#include <tuple>
#include <cstdint>
#include <type_traits>
#include <cstring>

namespace utools::code
{
    /// @brief 位数据打包器
    template <typename T>
    struct ByteHelper
    {
        /// @brief 向std::vector<uint8_t>中添加数据
        /// @param vec 目标vector
        /// @param value 需要增加的值
        static void append_bytes(std::vector<uint8_t> &vec, T value)
        {
            static_assert(std::is_integral<T>::value || std::is_floating_point<T>::value, "Only integral and floating point types are supported");
            if constexpr (std::is_integral<T>::value)
            {
                for (size_t i = 0; i < sizeof(T); ++i)
                {
                    vec.push_back((value >> (i * 8)) & 0xFF);
                }
            }
            else if constexpr (std::is_floating_point<T>::value)
            {
                uint8_t bytes[sizeof(T)];
                std::memcpy(bytes, &value, sizeof(T));
                vec.insert(vec.end(), bytes, bytes + sizeof(T));
            }
        }

        /// @brief 解包数据
        /// @param vec 目标vector
        /// @param offset 偏移量
        /// @return 解包后的数据
        static T extract_bytes(const std::vector<uint8_t> &vec, size_t &offset)
        {
            T value;
            if constexpr (std::is_integral<T>::value)
            {
                value = 0;
                for (size_t i = 0; i < sizeof(T); ++i)
                {
                    value |= static_cast<T>(vec[offset + i]) << (i * 8);
                }
            }
            else if constexpr (std::is_floating_point<T>::value)
            {
                std::memcpy(&value, vec.data() + offset, sizeof(T));
            }
            offset += sizeof(T);
            return value;
        }
    };

    /// @brief uint8_t的特化
    template <>
    struct ByteHelper<uint8_t>
    {
        static void append_bytes(std::vector<uint8_t> &vec, uint8_t value)
        {
            vec.push_back(value);
        }

        static uint8_t extract_bytes(const std::vector<uint8_t> &vec, size_t &offset)
        {
            return vec[offset++];
        }
    };

    /// @brief 打包器功能
    /// @tparam ...Args 打包的目标数据类型
    template <typename... Args>
    class BytePacker
    {
    public:
        static std::vector<uint8_t> pack(Args... args)
        {
            std::vector<uint8_t> result;
            (ByteHelper<Args>::append_bytes(result, args), ...);
            return result;
        }
    };

    /// @brief 解包器功能
    /// @tparam ...Args 解包的目标数据类型
    template <typename... Args>
    class ByteUnpacker
    {
    public:
        static std::tuple<Args...> unpack(const std::vector<uint8_t> &vec)
        {
            size_t offset = 0;
            return unpack_impl<Args...>(vec, offset);
        }

    private:
        template <typename T, typename... Rest>
        static std::tuple<T, Rest...> unpack_impl(const std::vector<uint8_t> &vec, size_t &offset)
        {
            T value = ByteHelper<T>::extract_bytes(vec, offset);
            if constexpr (sizeof...(Rest) == 0)
            {
                return std::make_tuple(value);
            }
            else
            {
                return std::tuple_cat(std::make_tuple(value), unpack_impl<Rest...>(vec, offset));
            }
        }
    };
}

#endif // __UTOOLS_BYTE_PACKER_H__