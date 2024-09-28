#ifndef __LEAST_SQUARE_METHOD_HPP__
#define __LEAST_SQUARE_METHOD_HPP__

#include <type_traits>
#include <vector>
#include <tuple>
namespace utools::fliter
{
    template <typename _ValType, typename = std::enable_if<std::is_floating_point<_ValType>::value>>
    class LeastSquareMethod
    {
    private:
        std::vector<std::pair<_ValType, _ValType>> __data; // 数据

    public:
        inline std::vector<std::pair<_ValType, _ValType>> &data()
        {
            return __data;
        }

        void push(const _ValType x, const _ValType y)
        {
            __data.push_back(std::make_pair(x, y));
        }

        /// @brief 线性拟合
        /// @return [k, b]
        const std::tuple<_ValType, _ValType> linear_fit()
        {
            _ValType sy{0};
            _ValType sx{0};
            _ValType sxx{0};
            _ValType sxy{0};
            for (auto [_x, _y] : __data)
            {
                sx += _x;       // 计算xi的和
                sy += _y;       // 计算yi的和
                sxx += _x * _x; // 计算xi的平方的和
                sxy += _x * _y; // 计算xi乘yi的的和
            }
            auto n{__data.size()};
            auto temp = n * sxx - sx * sx; // 方程的系数行列式
            auto temp0 = sy * sxx - sx * sxy;
            auto temp1 = n * sxy - sy * sx;
            return std::make_tuple<_ValType, _ValType>(temp0 / temp, temp1 / temp);
        }

        /// @brief 二次拟合
        /// @return [a, b, c]
        const std::tuple<_ValType, _ValType, _ValType> quadratic_fit()
        {
            _ValType sy{0};
            _ValType sx{0};
            _ValType sxx{0};
            _ValType sxy{0};
            _ValType sxxy{0};
            _ValType sxxx{0};
            _ValType sxxxx{0}; // 定义相关变量
            for (auto [_x, _y] : __data)
            {
                sx += _x;             // 计算xi的和
                sy += _y;             // 计算yi的和
                sxx += _x * _x;       // 计算xi的平方的和
                sxxx += pow(_x, 3);   // 计算xi的立方的和
                sxxxx += pow(_x, 4);  // 计算xi的4次方的和
                sxy += _x * _y;       // 计算xi乘yi的的和
                sxxy += _x * _x * _y; // 计算xi平方乘yi的和
            }
            auto n{__data.size()};
            auto temp = n * (sxx * sxxxx - sxxx * sxxx) - sx * (sx * sxxxx - sxx * sxxx) + sxx * (sx * sxxx - sxx * sxx);
            auto temp0 = sy * (sxx * sxxxx - sxxx * sxxx) - sxy * (sx * sxxxx - sxx * sxxx) + sxxy * (sx * sxxx - sxx * sxx);
            auto temp1 = n * (sxy * sxxxx - sxxy * sxxx) - sx * (sy * sxxxx - sxx * sxxy) + sxx * (sy * sxxx - sxy * sxx);
            auto temp2 = n * (sxx * sxxy - sxy * sxxx) - sx * (sx * sxxy - sy * sxxx) + sxx * (sx * sxy - sy * sxx);
            return std::make_tuple<_ValType, _ValType, _ValType>(temp0 / temp, temp1 / temp, temp2 / temp);
        }

        void reint()
        {
            __data.clear();
        }
    };
}

#endif
