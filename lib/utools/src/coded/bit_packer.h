/// @brief 该函数可以根据传入的模板参数（包括位宽和返回值的数据类型）将多个段拼接成一个完整的数据。

#ifndef __UTOOLS_BIT_PACKER_H__
#define __UTOOLS_BIT_PACKER_H__

#include <cstdint>
#include <tuple>
#include <cstddef>
#include <utility>

namespace utools::code
{
    // 手动实现 index_sequence 和 make_index_sequence
    template <std::size_t... Ints>
    struct index_sequence
    {
    };

    template <std::size_t N, std::size_t... Ints>
    struct make_index_sequence_helper : make_index_sequence_helper<N - 1, N - 1, Ints...>
    {
    };

    template <std::size_t... Ints>
    struct make_index_sequence_helper<0, Ints...>
    {
        using type = index_sequence<Ints...>;
    };

    template <std::size_t N>
    using make_index_sequence = typename make_index_sequence_helper<N>::type;

    // sum_bit_widths_up_to 前向声明
    template <size_t... BitWidths>
    constexpr size_t sum_bit_widths_up_to(size_t index);

    /// @brief 该函数可以根据传入的模板参数（包括位宽和返回值的数据类型）将多个段拼接成一个完整的数据
    /// @tparam T 返回值的数据类型
    /// @tparam BitWidths 各段位宽
    template <typename T, size_t... BitWidths>
    class BitPacker
    {
    public:
        static_assert(sizeof...(BitWidths) > 0, "At least one bit width must be specified");

        template <typename... Args>
        static T pack(Args... args)
        {
            static_assert(sizeof...(Args) == sizeof...(BitWidths), "Number of arguments must match number of bit widths");

            return pack_impl(std::make_tuple(args...), make_index_sequence<sizeof...(BitWidths)>{});
        }

    private:
        template <typename Tuple, size_t... Indices>
        static T pack_impl(const Tuple &values, index_sequence<Indices...>)
        {
            return pack_impl_helper<T, Tuple, sizeof...(Indices)>(values);
        }

        template <typename ResultType, typename Tuple, std::size_t N>
        struct pack_impl_helper
        {
            static ResultType pack(const Tuple &values)
            {
                return (std::get<N - 1>(values) << sum_bit_widths_up_to<BitWidths...>(N - 1)) |
                       pack_impl_helper<ResultType, Tuple, N - 1>::pack(values);
            }
        };

        template <typename ResultType, typename Tuple>
        struct pack_impl_helper<ResultType, Tuple, 0>
        {
            static ResultType pack(const Tuple &)
            {
                return 0;
            }
        };
    };

    /// @brief 将BitPacker打包的数据解包到指定的类型中
    /// @tparam T 目前数据类型
    /// @tparam BitWidths 每个段的位宽
    template <typename T, size_t... BitWidths>
    class BitUnpacker
    {
    public:
        static_assert(sizeof...(BitWidths) > 0, "At least one bit width must be specified");

        static auto unpack(T value) -> decltype(unpack_impl(value, make_index_sequence<sizeof...(BitWidths)>{}))
        {
            return unpack_impl(value, make_index_sequence<sizeof...(BitWidths)>{});
        }

    private:
        template <size_t Index, size_t Width>
        static uint64_t extract_bits(T value)
        {
            constexpr size_t offset = sum_bit_widths_up_to<BitWidths...>(Index);
            constexpr T mask = ((static_cast<T>(1) << Width) - 1) << offset;
            return (value & mask) >> offset;
        }

        template <size_t... Indices>
        static auto unpack_impl(T value, index_sequence<Indices...>)
            -> decltype(std::make_tuple(extract_bits<Indices, BitWidths>(value)...))
        {
            return std::make_tuple(extract_bits<Indices, BitWidths>(value)...);
        }
    };

    /// @brief 辅助函数，用于计算给定索引之前的位宽总和
    /// @param index 要计算的索引
    template <size_t... BitWidths>
    constexpr size_t sum_bit_widths_up_to(size_t index)
    {
        constexpr size_t bit_widths[] = {BitWidths...};
        size_t sum = 0;
        for (size_t i = 0; i < index; ++i)
        {
            sum += bit_widths[i];
        }
        return sum;
    }
}
#endif // __UTOOLS_BIT_PACKER_H__