
/*
A、名称：中位值滤波法
B、方法：
    连续采样N次（N取奇数），把N次采样值按大小排列，
    取中间值为本次有效值。
C、优点：
    能有效克服因偶然因素引起的波动干扰；
    对温度、液位的变化缓慢的被测参数有良好的滤波效果。
D、缺点：
    对流量、速度等快速变化的参数不宜。
*/
#ifndef __MEDIAN_FLITER_HPP__
#define __MEDIAN_FLITER_HPP__

#include <array>
#include <numeric>

namespace utools::fliter
{
    /// @brief 中位值滤波法
    /// @tparam _InputType 输入出值的数据类型
    /// @tparam _BufSize 缓存的大小，必须是奇数，不然输出值可以不是预期值
    template <class _InputType, unsigned short _BufSize>
    class MedianFliter
    {
    protected:
        std::array<_InputType, static_cast<size_t>(_BufSize)> _buf; // 数据缓存
        unsigned short _buf_pos{0};                                 // 当前操作数据的位置
        unsigned short _mid_pos{(_BufSize - 1) / 2};                // 中间值所在的位置，对应容器中的序号

    public:
        MedianFliter(_InputType init_val = 0)
        {
            reinit(init_val);
        }

        /// @brief 读取当前的滤波结果
        /// @return 滤波结果
        inline const _InputType &calc()
        {
            std::sort(_buf.begin(), _buf.end(), std::less<_InputType>());
            return _buf.at(_mid_pos);
        }

        /// @brief 写入数据
        /// @param val 数据值
        inline void push(_InputType val)
        {
            this->_buf.at(++this->_buf_pos % _BufSize) = val;
        }

        /// @brief 增加新的数据，并返回计算结果
        /// @tparam _ReturnType 返回值类型
        /// @param val 输入值
        /// @return 滤波结果
        template <class _ReturnType>
        inline const _ReturnType calc(_InputType val)
        {
            this->push(val);
            return static_cast<_ReturnType>(calc());
        }

        /// @brief 增加新的数据，并返回计算结果，计算结果类型与输出数据一样
        /// @param val 输入值
        /// @return 滤波结果
        const _InputType calc(_InputType val)
        {
            push(val);
            return calc();
        }

        /// @brief 重新初始化缓存
        /// @param init_val 初始化值
        void reinit(_InputType init_val = 0)
        {
            this->_buf.fill(init_val); // 初始化数组
            this->_buf_pos = 0;
        }
    };
}
#endif
