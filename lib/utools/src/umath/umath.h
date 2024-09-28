#ifndef __UTOOLS_MATH_H__
#define __UTOOLS_MATH_H__

#include <utility>
#include <initializer_list>
#include <functional>
#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>

namespace utools::math
{
    /// @brief 判断两个浮点数是否接近相等，优化判断绝地精度，不成立时再判断相对进度
    /// @tparam T1 浮点数类型
    /// @tparam T2 浮点数类型
    /// @param a 第一个浮点数
    /// @param b 第二个浮点数
    /// @param rel_tol 相对误差容忍度，默认为浮点数类型的epsilon
    /// @param abs_tol 绝对误差容忍度，默认为浮点数类型的epsilon
    /// @return 如果两个浮点数接近相等，则返回true，否则返回false
    /// @note 在默认rel_tol和abs_tol下，可以认为两个浮点数接近相等
    template <typename T1, typename T2>
    bool is_close(T1 a, T2 b,
                  T1 rel_tol = std::numeric_limits<T1>::epsilon(),
                  T1 abs_tol = std::numeric_limits<T1>::epsilon())
    {
        // 确保参数是数字类型
        static_assert(std::is_arithmetic<T1>::value, "T1 must be a numeric type");
        static_assert(std::is_arithmetic<T2>::value, "T2 must be a numeric type");

        // 计算绝对差值
        T1 diff = std::fabs(a - b);
        // 检查绝对误差
        if (diff <= abs_tol)
        {
            return true;
        }
        // 计算最大绝对值
        T1 max_abs = std::max(std::fabs(a), std::fabs(b));

        // 检查相对误差
        return diff <= rel_tol * max_abs;
    }

    /// @brief 判断两个浮点数是否接近相等，采用绝对误差判断方式
    /// @tparam T1 浮点数类型
    /// @tparam T2 浮点数类型
    /// @param a 第一个浮点数
    /// @param b 第二个浮点数
    /// @param abs_tol 绝对误差容忍度，默认为浮点数类型的epsilon
    /// @return 如果两个浮点数接近相等，则返回true，否则返回false
    /// @note 在默认abs_tol下，可以认为两个浮点数接近相等
    template <typename T1, typename T2>
    inline bool is_close_abs(T1 a, T2 b, T1 abs_tol = std::numeric_limits<T1>::epsilon())
    {
        // 确保参数是数字类型
        static_assert(std::is_arithmetic<T1>::value, "T1 must be a numeric type");
        static_assert(std::is_arithmetic<T2>::value, "T2 must be a numeric type");

        return std::fabs(a - b) <= abs_tol;
    }

    /// @brief 判断两个浮点数是否接近相等，采用相对误差判断方式
    /// @tparam T1 浮点数类型
    /// @tparam T2 浮点数类型
    /// @param a 第一个浮点数
    /// @param b 第二个浮点数
    /// @param rel_tol 相对误差容忍度，默认为浮点数类型的epsilon
    /// @return 如果两个浮点数接近相等，则返回true，否则返回false
    /// @note 在默认rel_tol下，可以认为两个浮点数接近相等
    template <typename T1, typename T2>
    bool is_close_ref(T1 a, T2 b, T1 rel_tol = std::numeric_limits<T1>::epsilon())
    {
        // 确保参数是数字类型
        static_assert(std::is_arithmetic<T1>::value, "T1 must be a numeric type");
        static_assert(std::is_arithmetic<T2>::value, "T2 must be a numeric type");

        T1 diff = std::fabs(a - b);                        // 计算绝对差值
        T1 max_abs = std::max(std::fabs(a), std::fabs(b)); // 计算最大绝对值

        return diff <= rel_tol * max_abs;
    }

    /// @brief 将值限制在指定的范围内
    /// @tparam INPUT_TYP 输出数据类型
    /// @tparam OUTPUT_TYP 输出数据类型
    /// @param val 需要判断的值
    /// @param min 最小值
    /// @param max 最大值
    /// @return 限制在范围内的值
    template <typename INPUT_TYP, typename OUTPUT_TYP,
              typename = typename std::enable_if<std::is_arithmetic<INPUT_TYP>::value &&
                                                 std::is_arithmetic<OUTPUT_TYP>::value>::type>
    inline OUTPUT_TYP clamp(const OUTPUT_TYP &val, const INPUT_TYP &min, const INPUT_TYP &max)
    {
        using COMMON_TYP = typename std::common_type<INPUT_TYP, OUTPUT_TYP>::type;
        return static_cast<OUTPUT_TYP>(std::max(static_cast<COMMON_TYP>(min),
                                                std::min(static_cast<COMMON_TYP>(max),
                                                         static_cast<COMMON_TYP>(val))));
    }

    /// @brief 将值限制在指定的范围内
    /// @tparam INPUT_TYP 输入与输出数据类型
    /// @param val 需要判断的值
    /// @param min 最小值
    /// @param max 最大值
    /// @return 限制在范围内的值
    template <typename INPUT_TYP,
              typename = typename std::enable_if<std::is_arithmetic<INPUT_TYP>::value>::type>
    inline INPUT_TYP clamp(const INPUT_TYP &val, const INPUT_TYP &min, const INPUT_TYP &max)
    {
        return std::max(min, std::min(max, val));
    }

    /// @brief 将一个值从原始范围映射到目标范围
    /// @tparam DET_OUTPUT_TYP 目标范围的最小值和最大值类型
    /// @tparam SRC_INPUT_TYP 原始范围的最小值和最大值类型
    /// @tparam VAL_TYP 要映射的值类型
    /// @param val 要映射的值
    /// @param src_min 要映射的值最小值
    /// @param src_max 要映射的值最大值
    /// @param det_min 目标映射的值最小值
    /// @param det_max 目标映射的值最大值
    /// @return 映射后的值
    template <typename DET_OUTPUT_TYP, typename SRC_INPUT_TYP, typename VAL_TYP,
              typename = typename std::enable_if<std::is_arithmetic<SRC_INPUT_TYP>::value &&
                                                 std::is_arithmetic<DET_OUTPUT_TYP>::value &&
                                                 std::is_arithmetic<DET_OUTPUT_TYP>::value>::type>
    inline DET_OUTPUT_TYP linear_map(VAL_TYP &&val,
                                     const SRC_INPUT_TYP &src_min, const SRC_INPUT_TYP &src_max,
                                     const DET_OUTPUT_TYP &det_min, const DET_OUTPUT_TYP &det_max)
    {
        static_assert(std::is_arithmetic<SRC_INPUT_TYP>::value, "SRC_INPUT_TYP must be arithmetic");
        static_assert(std::is_arithmetic<DET_OUTPUT_TYP>::value, "DET_OUTPUT_TYP must be arithmetic");
        static_assert(std::is_arithmetic<typename std::decay<VAL_TYP>::type>::value, "VAL_TYP must be arithmetic");

        using COMMON_TYP = typename std::common_type<SRC_INPUT_TYP, DET_OUTPUT_TYP>::type;

        return static_cast<DET_OUTPUT_TYP>((static_cast<COMMON_TYP>(std::forward<VAL_TYP>(val)) - static_cast<COMMON_TYP>(src_min)) *
                                               (static_cast<COMMON_TYP>(det_max) - static_cast<COMMON_TYP>(det_min)) /
                                               (static_cast<COMMON_TYP>(src_max) - static_cast<COMMON_TYP>(src_min)) +
                                           static_cast<COMMON_TYP>(det_min));
    }

    /// @brief 将一个值从原始范围映射到目标范围
    /// @tparam DET_OUTPUT_TYP 目标范围的最小值和最大值类型
    /// @tparam SRC_INPUT_TYP 原始范围的最小值和最大值类型
    /// @tparam VAL_TYP 要映射的值类型
    /// @param val 要映射的值
    /// @param src_range 要映射的值的原始范围
    /// @param det_range 目标映射的值范围
    /// @return 映射后的值
    template <typename DET_OUTPUT_TYP, typename SRC_INPUT_TYP, typename VAL_TYP>
    inline DET_OUTPUT_TYP linear_map(VAL_TYP &&val,
                                     const std::pair<SRC_INPUT_TYP, SRC_INPUT_TYP> &src_range,
                                     const std::pair<DET_OUTPUT_TYP, DET_OUTPUT_TYP> &det_range)
    {
        return linear_map(
            std::forward<VAL_TYP>(val),
            src_range.first, src_range.second,
            det_range.first, det_range.second);
    }

    /// @brief 将一个值从原始范围映射到目标范围
    /// @tparam DET_OUTPUT_TYP 目标范围的最小值和最大值类型
    /// @tparam SRC_INPUT_TYP 原始范围的最小值和最大值类型
    /// @tparam VAL_TYP 要映射的值类型
    /// @param val 要映射的值
    /// @param src_range 要映射的值的原始范围
    /// @param det_range 目标映射的值范围
    /// @return 映射后的值
    template <typename DET_OUTPUT_TYP, typename SRC_INPUT_TYP, typename VAL_TYP>
    inline DET_OUTPUT_TYP linear_map(VAL_TYP &&val,
                                     const std::initializer_list<SRC_INPUT_TYP> &&src_range,
                                     const std::initializer_list<DET_OUTPUT_TYP> &&det_range)
    {
        return linear_map(
            std::forward<VAL_TYP>(val),
            *src_range.begin(), *(src_range.begin() + 1),
            *det_range.begin(), *(det_range.begin() + 1));
    }

    /// @brief 将一个值从原始范围映射到目标范围
    /// @tparam DET_OUTPUT_TYP 目标范围的最小值和最大值类型
    /// @tparam SRC_INPUT_TYP 原始范围的最小值和最大值类型
    /// @tparam VAL_TYP 要映射的值类型
    /// @param val 要映射的值
    /// @param src_range 要映射的值的原始范围
    /// @param det_range 目标映射的值范围
    /// @return 映射后的值
    template <typename DET_OUTPUT_TYP, typename SRC_INPUT_TYP, typename VAL_TYP>
    inline DET_OUTPUT_TYP linear_map(VAL_TYP &val,
                                     const std::initializer_list<SRC_INPUT_TYP> &&src_range,
                                     const std::initializer_list<DET_OUTPUT_TYP> &&det_range)
    {
        return linear_map(
            std::forward<VAL_TYP>(val),
            *src_range.begin(), *(src_range.begin() + 1),
            *det_range.begin(), *(det_range.begin() + 1));
    }

    /// @brief 将一个值从原始范围映射到目标范围，输入数据中间不连续，不连续部分映射到具体的点
    /// @tparam DET_OUTPUT_TYP 目标输出类型
    /// @tparam SRC_INPUT_TYP 输入类型
    /// @tparam VAL_TYP 输入值类型
    /// @param value 输入值
    /// @param input_range 输入范围
    /// @param output_range 输出范围
    /// @param discontinuous_range 不连续的范围
    /// @param align_point 不连续范围对应的点
    /// @return 映射后的值
    template <typename DET_OUTPUT_TYP, typename SRC_INPUT_TYP, typename VAL_TYP,
              typename = typename std::enable_if<std::is_arithmetic<SRC_INPUT_TYP>::value &&
                                                 std::is_arithmetic<DET_OUTPUT_TYP>::value &&
                                                 std::is_arithmetic<VAL_TYP>::value>::type>
    DET_OUTPUT_TYP linear_map_dc2c_ap(
        VAL_TYP &&value,
        std::pair<SRC_INPUT_TYP, SRC_INPUT_TYP> input_range,
        std::pair<DET_OUTPUT_TYP, DET_OUTPUT_TYP> output_range,
        std::pair<SRC_INPUT_TYP, SRC_INPUT_TYP> discontinuous_range,
        DET_OUTPUT_TYP &&align_point)
    {
        using COMMON_TYP = typename std::common_type<SRC_INPUT_TYP, DET_OUTPUT_TYP>::type;

        COMMON_TYP input_min = static_cast<COMMON_TYP>(input_range.first);
        COMMON_TYP input_max = static_cast<COMMON_TYP>(input_range.second);
        COMMON_TYP output_min = static_cast<COMMON_TYP>(output_range.first);
        COMMON_TYP output_max = static_cast<COMMON_TYP>(output_range.second);
        COMMON_TYP discontinuous_min = static_cast<COMMON_TYP>(discontinuous_range.first);
        COMMON_TYP discontinuous_max = static_cast<COMMON_TYP>(discontinuous_range.second);

        COMMON_TYP mapped_value;
        // 计算对齐点对应的输出值
        COMMON_TYP align_output = ((double)(align_point - input_min) / (double)(input_max - input_min)) *
                                      (output_max - output_min) +
                                  output_min;
        // 映射对齐点左边的部分
        if (value < discontinuous_min)
        {
            // 映射到不连续区间之前的部分
            mapped_value = ((double)(value - input_min) / (double)(discontinuous_min - input_min)) *
                               (align_output - output_min) +
                           output_min;
        }
        else if (value > discontinuous_max)
        {
            // 映射到不连续区间之后的部分
            mapped_value = ((double)(value - discontinuous_max) / (double)(input_max - discontinuous_max)) *
                               (output_max - align_output) +
                           align_output;
        }
        else
        {
            mapped_value = align_output; // 映射到不连续区间内部的部分
        }

        return static_cast<DET_OUTPUT_TYP>(mapped_value);
    }

    /// @brief 将一个值从原始范围映射到目标范围，输入数据中间不连续，不连续部分映射到具体的点
    /// @tparam DET_OUTPUT_TYP 目标输出类型
    /// @tparam SRC_INPUT_TYP 输入类型
    /// @tparam VAL_TYP 输入值类型
    /// @param value 输入值
    /// @param input_range 输入范围
    /// @param output_range 输出范围
    /// @param discontinuous_range 不连续的范围
    /// @param align_point 不连续范围对应的点
    /// @return 映射后的值
    template <typename DET_OUTPUT_TYP, typename SRC_INPUT_TYP, typename VAL_TYP>
    DET_OUTPUT_TYP linear_map_dc2c_ap(
        VAL_TYP &&value,
        std::initializer_list<SRC_INPUT_TYP> input_range,
        std::initializer_list<DET_OUTPUT_TYP> output_range,
        std::initializer_list<SRC_INPUT_TYP> discontinuous_range,
        DET_OUTPUT_TYP &&align_point)
    {
        return linear_map_dc2c_ap<DET_OUTPUT_TYP, SRC_INPUT_TYP>(
            std::forward<VAL_TYP>(value),
            std::make_pair(*input_range.begin(), *(input_range.begin() + 1)),
            std::make_pair(*output_range.begin(), *(output_range.begin() + 1)),
            std::make_pair(*discontinuous_range.begin(), *(discontinuous_range.begin() + 1)),
            align_point);
    }

    /// @brief 将一个值从原始范围映射到目标范围，输入数据中间不连续，不连续部分映射到具体的点
    /// @tparam DET_OUTPUT_TYP 目标输出类型
    /// @tparam SRC_INPUT_TYP 输入类型
    /// @tparam VAL_TYP 输入值类型
    /// @param value 输入值
    /// @param input_range 输入范围
    /// @param output_range 输出范围
    /// @param discontinuous_range 不连续的范围
    /// @param align_point 不连续范围对应的点
    /// @return 映射后的值
    template <typename DET_OUTPUT_TYP, typename SRC_INPUT_TYP, typename VAL_TYP>
    DET_OUTPUT_TYP linear_map_dc2c_ap(
        VAL_TYP &value,
        std::initializer_list<SRC_INPUT_TYP> input_range,
        std::initializer_list<DET_OUTPUT_TYP> output_range,
        std::initializer_list<SRC_INPUT_TYP> discontinuous_range,
        DET_OUTPUT_TYP &&align_point)
    {
        return linear_map_dc2c_ap<DET_OUTPUT_TYP, SRC_INPUT_TYP>(
            std::forward<VAL_TYP>(value),
            std::make_pair(*input_range.begin(), *(input_range.begin() + 1)),
            std::make_pair(*output_range.begin(), *(output_range.begin() + 1)),
            std::make_pair(*discontinuous_range.begin(), *(discontinuous_range.begin() + 1)),
            align_point);
    }

    /// @brief 将一个值从原始范围映射到目标范围，输入数据中间不连续
    /// @tparam DET_OUTPUT_TYP 目标输出类型
    /// @tparam SRC_INPUT_TYP 输入类型
    /// @tparam VAL_TYP 输入值类型
    /// @param value 输入值
    /// @param input_range 输入范围
    /// @param output_range 输出范围
    /// @param discontinuous_range 不连续的范围
    /// @return 映射后的值
    template <typename DET_OUTPUT_TYP, typename SRC_INPUT_TYP, typename VAL_TYP,
              typename = typename std::enable_if<std::is_arithmetic<SRC_INPUT_TYP>::value &&
                                                 std::is_arithmetic<DET_OUTPUT_TYP>::value &&
                                                 std::is_arithmetic<VAL_TYP>::value>::type>
    DET_OUTPUT_TYP linear_map_dc2c(
        VAL_TYP &&value,
        std::pair<SRC_INPUT_TYP, SRC_INPUT_TYP> input_range,
        std::pair<DET_OUTPUT_TYP, DET_OUTPUT_TYP> output_range,
        std::pair<SRC_INPUT_TYP, SRC_INPUT_TYP> discontinuous_range)
    {
        using COMMON_TYP = typename std::common_type<SRC_INPUT_TYP, DET_OUTPUT_TYP>::type;

        COMMON_TYP input_min = static_cast<COMMON_TYP>(input_range.first);
        COMMON_TYP input_max = static_cast<COMMON_TYP>(input_range.second);
        COMMON_TYP output_min = static_cast<COMMON_TYP>(output_range.first);
        COMMON_TYP output_max = static_cast<COMMON_TYP>(output_range.second);
        COMMON_TYP discontinuous_min = static_cast<COMMON_TYP>(discontinuous_range.first);
        COMMON_TYP discontinuous_max = static_cast<COMMON_TYP>(discontinuous_range.second);

        COMMON_TYP mapped_value;
        // 计算对齐点
        COMMON_TYP align_point = (discontinuous_min + discontinuous_max) / 2;
        // 计算对齐点对应的输出值
        COMMON_TYP align_output = ((double)(align_point - input_min) / (double)(input_max - input_min)) *
                                      (output_max - output_min) +
                                  output_min;
        // 映射对齐点左边的部分
        if (value < discontinuous_min)
        {
            // 映射到不连续区间之前的部分
            mapped_value = ((double)(value - input_min) / (double)(discontinuous_min - input_min)) *
                               (align_output - output_min) +
                           output_min;
        }
        else if (value > discontinuous_max)
        {
            // 映射到不连续区间之后的部分
            mapped_value = ((double)(value - discontinuous_max) / (double)(input_max - discontinuous_max)) *
                               (output_max - align_output) +
                           align_output;
        }
        else
        {
            mapped_value = align_output; // 映射到不连续区间内部的部分
        }

        return static_cast<DET_OUTPUT_TYP>(mapped_value);
    }

    /// @brief 将一个值从原始范围映射到目标范围，输入数据中间不连续
    /// @tparam DET_OUTPUT_TYP 目标输出类型
    /// @tparam SRC_INPUT_TYP 输入类型
    /// @tparam VAL_TYP 输入值类型
    /// @param value 输入值
    /// @param input_range 输入范围
    /// @param output_range 输出范围
    /// @param discontinuous_range 不连续的范围
    /// @return 映射后的值
    template <typename DET_OUTPUT_TYP, typename SRC_INPUT_TYP, typename VAL_TYP>
    DET_OUTPUT_TYP linear_map_dc2c(
        VAL_TYP &&value,
        std::initializer_list<SRC_INPUT_TYP> input_range,
        std::initializer_list<DET_OUTPUT_TYP> output_range,
        std::initializer_list<SRC_INPUT_TYP> discontinuous_range)
    {
        return linear_map_dc2c<DET_OUTPUT_TYP, SRC_INPUT_TYP>(
            std::forward<VAL_TYP>(value),
            std::make_pair(*input_range.begin(), *(input_range.begin() + 1)),
            std::make_pair(*output_range.begin(), *(output_range.begin() + 1)),
            std::make_pair(*discontinuous_range.begin(), *(discontinuous_range.begin() + 1)));
    }

    /// @brief 将一个值从原始范围映射到目标范围，输入数据中间不连续
    /// @tparam DET_OUTPUT_TYP 目标输出类型
    /// @tparam SRC_INPUT_TYP 输入类型
    /// @tparam VAL_TYP 输入值类型
    /// @param value 输入值
    /// @param input_range 输入范围
    /// @param output_range 输出范围
    /// @param discontinuous_range 不连续的范围
    /// @return 映射后的值
    template <typename DET_OUTPUT_TYP, typename SRC_INPUT_TYP, typename VAL_TYP>
    DET_OUTPUT_TYP linear_map_dc2c(
        VAL_TYP &value,
        std::initializer_list<SRC_INPUT_TYP> input_range,
        std::initializer_list<DET_OUTPUT_TYP> output_range,
        std::initializer_list<SRC_INPUT_TYP> discontinuous_range)
    {
        return linear_map_dc2c<DET_OUTPUT_TYP, SRC_INPUT_TYP>(
            std::forward<VAL_TYP>(value),
            std::make_pair(*input_range.begin(), *(input_range.begin() + 1)),
            std::make_pair(*output_range.begin(), *(output_range.begin() + 1)),
            std::make_pair(*discontinuous_range.begin(), *(discontinuous_range.begin() + 1)));
    }

    /// @brief 分段性能映射生成器，返回一个可调用对象，可能通过向其传递参数来生成分段性能映射后的结果
    /// @tparam SRC_INPUT_TYP 要映射的值的原始范围的类型
    /// @tparam DET_OUTPUT_TYP 目标映射的值范围的类型
    /// @param src 要映射的值的原始范围
    /// @param det 目标映射的值范围
    /// @return 分段性能映射生成器
    template <typename SRC_INPUT_TYP, typename DET_OUTPUT_TYP>
    auto piecewise_linear_generator(const std::vector<SRC_INPUT_TYP> &src,
                                    const std::vector<DET_OUTPUT_TYP> &det)
        -> std::function<DET_OUTPUT_TYP(SRC_INPUT_TYP)>
    {
        // 检查src和det的大小是否匹配
        if (src.size() != det.size() || src.size() < 2)
        {
            throw std::invalid_argument("src_range and det_range must have the same size and at least two elements.");
        }

        // 排序src和det
        std::vector<std::pair<SRC_INPUT_TYP, DET_OUTPUT_TYP>> pairs;
        for (size_t i = 0; i < src.size(); ++i)
        {
            pairs.emplace_back(src[i], det[i]);
        }
        std::sort(pairs.begin(), pairs.end());

        return [pairs](SRC_INPUT_TYP x) -> DET_OUTPUT_TYP
        {
            // 如果x在范围外，返回边界值
            if (x <= pairs.front().first)
            {
                return static_cast<DET_OUTPUT_TYP>(pairs.front().second);
            }
            if (x >= pairs.back().first)
            {
                return static_cast<DET_OUTPUT_TYP>(pairs.back().second);
            }

            // 查找x所在的段
            auto it = std::upper_bound(pairs.begin(), pairs.end(), std::make_pair(x, DET_OUTPUT_TYP()),
                                       [](const std::pair<SRC_INPUT_TYP, DET_OUTPUT_TYP> &lhs, const std::pair<SRC_INPUT_TYP, DET_OUTPUT_TYP> &rhs)
                                       {
                                           return lhs.first < rhs.first;
                                       });

            // 线性插值
            const auto &[x1, y1] = *(it - 1);
            const auto &[x2, y2] = *it;
            DET_OUTPUT_TYP y = static_cast<DET_OUTPUT_TYP>(y1 + (y2 - y1) * (x - x1) / (x2 - x1));
            return y;
        };
    }

    /// @brief 分段性能映射生成器，返回一个可调用对象，可能通过向其传递参数来生成分段性能映射后的结果
    /// @param src_range 要映射的值的原始范围
    /// @param det_range 目标映射的值范围
    /// @return 分段性能映射生成器
    template <typename SRC_INPUT_TYP, typename DET_OUTPUT_TYP, typename RETURN_TYP>
    auto piecewise_linear_generator(const std::initializer_list<SRC_INPUT_TYP> &src_range,
                                    const std::initializer_list<DET_OUTPUT_TYP> *det_range)
        -> std::function<const RETURN_TYP(SRC_INPUT_TYP)>
    {
        // 将initializer_list转换为std::vector以便于处理
        std::vector<SRC_INPUT_TYP> src(src_range);
        std::vector<DET_OUTPUT_TYP> det(det_range);
        return piecewise_linear_generator(src, det);
    }
}

#endif // __UTOOLS_MATH_H__